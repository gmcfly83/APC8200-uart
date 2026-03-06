#include <ESP8266WiFi.h>
#include <ESP8266WebServer.h>
#include <ESP8266mDNS.h>
#include <EEPROM.h>
#include <SoftwareSerial.h>

// ===================== SENSOR SERIAL (D1 mini / ESP8266) =====================
#define SENSOR_RX_PIN D5   // GPIO14  (connect to sensor TX)
#define SENSOR_TX_PIN D6   // GPIO12  (optional, connect to sensor RX)
#define SENSOR_BAUD   19200

SoftwareSerial SensorSer(SENSOR_RX_PIN, SENSOR_TX_PIN, false);

// ===================== WIFI PROVISIONING STORAGE =====================
#define MAX_NETWORKS 8
#define MAX_SSID_LEN 24
#define MAX_PASS_LEN 24
#define EEPROM_SIZE (MAX_NETWORKS * (MAX_SSID_LEN + MAX_PASS_LEN))

#define AP_SSID "CO2Monitor-AP"
#define AP_PASSWORD "1through8"

ESP8266WebServer server(80);

volatile float    co2ppm      = NAN;
volatile float    temperature = NAN;
volatile uint32_t statusVal   = 0;
volatile uint32_t errorMask   = 0;
volatile uint32_t rVal        = 0;

struct WifiCred {
  char ssid[MAX_SSID_LEN];
  char pass[MAX_PASS_LEN];
};
WifiCred wifiList[MAX_NETWORKS];
int wifiCount = 0;

// ===================== EEPROM =====================
void loadWifiCreds() {
  EEPROM.begin(EEPROM_SIZE);
  wifiCount = 0;

  for (int i = 0; i < MAX_NETWORKS; i++) {
    int base = i * (MAX_SSID_LEN + MAX_PASS_LEN);

    for (int j = 0; j < MAX_SSID_LEN; j++) wifiList[i].ssid[j] = EEPROM.read(base + j);
    for (int j = 0; j < MAX_PASS_LEN; j++) wifiList[i].pass[j] = EEPROM.read(base + MAX_SSID_LEN + j);

    if (wifiList[i].ssid[0] != 0 && wifiList[i].ssid[0] != (char)0xFF) wifiCount++;
  }

  Serial.printf("Loaded %d Wi-Fi networks\n", wifiCount);
}

void saveWifiCreds() {
  for (int i = 0; i < MAX_NETWORKS; i++) {
    int base = i * (MAX_SSID_LEN + MAX_PASS_LEN);

    for (int j = 0; j < MAX_SSID_LEN; j++) EEPROM.write(base + j, wifiList[i].ssid[j]);
    for (int j = 0; j < MAX_PASS_LEN; j++) EEPROM.write(base + MAX_SSID_LEN + j, wifiList[i].pass[j]);
  }
  EEPROM.commit();
}

// ===================== CONNECT WIFI (STA then AP) =====================
void connectWiFi() {
  bool connected = false;

  WiFi.mode(WIFI_STA);
  WiFi.persistent(false);

  for (int i = 0; i < MAX_NETWORKS; i++) {
    if (wifiList[i].ssid[0] == 0 || wifiList[i].ssid[0] == (char)0xFF) continue;

    Serial.printf("Trying Wi-Fi: %s\n", wifiList[i].ssid);
    WiFi.begin(wifiList[i].ssid, wifiList[i].pass);

    unsigned long start = millis();
    while (WiFi.status() != WL_CONNECTED && millis() - start < 10000) {
      delay(10);
      yield();
    }

    if (WiFi.status() == WL_CONNECTED) {
      connected = true;
      Serial.printf("Connected to %s\n", wifiList[i].ssid);
      break;
    }
  }

  if (!connected) {
    WiFi.mode(WIFI_AP);
    WiFi.softAP(AP_SSID, AP_PASSWORD);
    Serial.println("Started AP for provisioning");
    Serial.print("AP IP: "); Serial.println(WiFi.softAPIP());
  } else {
    Serial.print("STA IP: "); Serial.println(WiFi.localIP());
    if (MDNS.begin("co2sensor")) {
      Serial.println("mDNS responder started: http://co2sensor.local");
    }
  }
}

// ===================== SENSOR PARSER =====================
// 0x02 + [S/C/R/E/T] + 6 ASCII hex + 0x8D (or 0x0D)
// Noise: 0x80 padding, and 0x8D high-bit CR.
// We normalize: if b==0x80 or 0x8D => b &= 0x7F
// And require exactly 6 hex digits to ignore partial junk.

struct Frame {
  bool gotS=false, gotC=false, gotR=false, gotE=false, gotT=false;
  uint32_t S=0, R=0, E=0;
  float Cppm=0, Tc=0;
};
static Frame fr;

static inline bool isHexChar(uint8_t c){
  return (c>='0'&&c<='9') || (c>='A'&&c<='F') || (c>='a'&&c<='f');
}
static inline uint8_t toUpperHex(uint8_t c){
  return (c>='a'&&c<='f') ? (c-32) : c;
}
static uint32_t parseHex6(const char *buf){
  uint32_t v=0;
  for(int i=0;i<6;i++){
    char c = buf[i];
    v <<= 4;
    if(c>='0'&&c<='9') v |= (c-'0');
    else v |= (toUpperHex(c)-'A'+10);
  }
  return v;
}
static int32_t signExtend24(uint32_t v){
  if(v & 0x800000) return (int32_t)(v | 0xFF000000);
  return (int32_t)v;
}

static void clearFrame(){ fr = Frame(); }

static void applyField(char t, uint32_t raw24){
  switch(t){
    case 'S': fr.S = raw24; fr.gotS = true; break;
    case 'R': fr.R = raw24; fr.gotR = true; break;
    case 'E': fr.E = raw24; fr.gotE = true; break;

    // CO2: 24.8 fixed point
    case 'C': fr.Cppm = (float)raw24 / 256.0f; fr.gotC = true; break;

    case 'T': {
      int32_t s24 = signExtend24(raw24);
      fr.Tc = (float)s24 / 131072.0f;
      fr.gotT = true;
    } break;
  }
}

static void publishIfReady(){
  if(fr.gotC) co2ppm = fr.Cppm;

  if(fr.gotT) temperature = fr.Tc;
  if(fr.gotS) statusVal = fr.S;
  if(fr.gotE) errorMask = fr.E;
  if(fr.gotR) rVal = fr.R;

  // Optional: when full frame is present, print one line
  if(fr.gotS && fr.gotC && fr.gotR && fr.gotE && fr.gotT){
    Serial.printf("CO2=%.2f ppm  T=%.2f C  S=%06lX R=%06lX E=%06lX\n",
      (double)co2ppm, (double)temperature,
      (unsigned long)statusVal, (unsigned long)rVal, (unsigned long)errorMask
    );
    clearFrame();
  }
}

static void feedByte(uint8_t b){
  if(b == 0x80 || b == 0x8D) b &= 0x7F;  // normalize padding + CR-highbit

  static bool inField=false;
  static char type=0;
  static char hexBuf[7];
  static int  hexIdx=0;

  if(b == 0x02){ // STX
    inField = true;
    type = 0;
    hexIdx = 0;
    return;
  }
  if(!inField) return;

  if(type == 0){
    if(b=='S' || b=='C' || b=='R' || b=='E' || b=='T'){
      type = (char)b;
    } else {
      inField = false;
    }
    return;
  }

  if(b == '\r' || b == '\n'){
    if(hexIdx == 6){
      hexBuf[6] = 0;
      uint32_t raw24 = parseHex6(hexBuf);
      applyField(type, raw24);
      publishIfReady();
    }
    inField = false;
    return;
  }

  if(isHexChar(b)){
    if(hexIdx < 6) hexBuf[hexIdx++] = (char)toUpperHex(b);
    return;
  }

  if(b == 0x00) return; // ignore null padding

  // Anything else inside a field => abort/resync
  inField = false;
}

// ===================== WEB HANDLERS =====================
void handleRoot();
void handleData();
void handleNetworks();
void handleAddNetwork();
void handleRemoveNetwork();

void handleRoot() {
  String page = R"rawliteral(
<!DOCTYPE html>
<html>
<head>
<title>CO2 Monitor</title>
<style>
body { background-color:#121212;color:#EEE;font-family:sans-serif;text-align:center; }
h1 {font-size: 3.0rem; color: #8C4A11;}
.value { font-size:3em;margin:20px; color: #00ADD6;}
.label { font-size: 1em; color: #C7F062;}
input { margin:5px; padding:5px; width:200px; background-color: #D98B2B;}
button { padding:5px 10px; background: #8C4A11;}
h2 {color: #D98B2B;}
</style>
</head>
<body>
<h1>APC8200 CO&#8322; Monitor</h1>
<div>
  <div class="value" id="co2">---</div>
 <div class="label">CO&#8322; (ppm)</div>
</div>
<div>
  <div class="value" id="temp">---</div>
   <div class="label">Temperature (&deg;C)</div>
</div>
<div>
  <div class="value" id="status">---</div>
  <div class="label">Status</div>
</div>
<div>
  <div class="value" id="error">---</div>
  <div class="label">Error Mask</div>
</div>

<h2>Wi-Fi Provisioning</h2>
<div id="wifiDiv">
<form id="wifiForm">
SSID:<br><input type="text" id="ssid" maxlength=24><br>
Password:<br><input type="text" id="pass" maxlength=24><br>
<button type="button" onclick="addWifi()">Add Network</button>
</form>
<h3>Stored Networks</h3>
<ul id="networkList"></ul>
</div>

<script>
async function update() {
  const resp = await fetch('/data');
  const data = await resp.json();
  document.getElementById('co2').innerText = (data.co2==null) ? "---" : data.co2.toFixed(1);
  document.getElementById('temp').innerText = (data.temp==null) ? "---" : data.temp.toFixed(1);
  document.getElementById('status').innerText = data.status;
  document.getElementById('error').innerText = data.error;
}

async function loadNetworks() {
  const resp = await fetch('/networks');
  const data = await resp.json();
  const list = document.getElementById('networkList');
  list.innerHTML = '';
  data.forEach((item, idx) => {
    let li = document.createElement('li');
    li.innerText = item.ssid + " ";
    let btn = document.createElement('button');
    btn.innerText = 'Remove';
    btn.onclick = async ()=>{await fetch('/remove?idx='+idx); loadNetworks();}
    li.appendChild(btn);
    list.appendChild(li);
  });
}

async function addWifi() {
  const ssid = document.getElementById('ssid').value;
  const pass = document.getElementById('pass').value;
  await fetch('/add?ssid='+encodeURIComponent(ssid)+'&pass='+encodeURIComponent(pass));
  document.getElementById('ssid').value='';
  document.getElementById('pass').value='';
  loadNetworks();
}

setInterval(update,1000);
update();
loadNetworks();
</script>
</body>
</html>
)rawliteral";

  server.send(200, "text/html", page);
}

void handleData() {
  String json = "{";
  json += "\"co2\":" + (isnan(co2ppm) ? String("null") : String(co2ppm, 1)) + ",";
  json += "\"temp\":" + (isnan(temperature) ? String("null") : String(temperature, 1)) + ",";
  json += "\"status\":\"" + String(statusVal, HEX) + "\",";
  json += "\"error\":\"" + String(errorMask, HEX) + "\"";
  json += "}";
  server.send(200, "application/json", json);
}

void handleNetworks() {
  // NOTE: This returns "slots that are used" as a compact list.
  String json = "[";
  bool first = true;
  for (int i = 0; i < MAX_NETWORKS; i++) {
    if (wifiList[i].ssid[0] != 0 && wifiList[i].ssid[0] != (char)0xFF) {
      if (!first) json += ",";
      json += "{\"ssid\":\""+String(wifiList[i].ssid)+"\",\"slot\":"+String(i)+"}";
      first=false;
    }
  }
  json += "]";
  server.send(200, "application/json", json);
}

void handleAddNetwork() {
  if (!server.hasArg("ssid") || !server.hasArg("pass")) {
    server.send(400,"text/plain","Missing ssid or pass");
    return;
  }

  String ssid = server.arg("ssid");
  String pass = server.arg("pass");

  for (int i = 0; i < MAX_NETWORKS; i++) {
    if (wifiList[i].ssid[0]==0 || wifiList[i].ssid[0]==(char)0xFF) {
      memset(wifiList[i].ssid, 0, MAX_SSID_LEN);
      memset(wifiList[i].pass, 0, MAX_PASS_LEN);
      strncpy(wifiList[i].ssid, ssid.c_str(), MAX_SSID_LEN-1);
      strncpy(wifiList[i].pass, pass.c_str(), MAX_PASS_LEN-1);

      saveWifiCreds();
      server.send(200,"text/plain","Added. Rebooting...");
      delay(500);
      ESP.restart();
      return;
    }
  }
  server.send(400,"text/plain","No empty slots");
}

void handleRemoveNetwork() {
  if (!server.hasArg("idx")) { server.send(400,"text/plain","Missing index"); return; }
  int idx = server.arg("idx").toInt();
  if (idx<0 || idx>=MAX_NETWORKS) { server.send(400,"text/plain","Invalid index"); return; }

  wifiList[idx].ssid[0]=0;
  wifiList[idx].pass[0]=0;
  saveWifiCreds();

  server.send(200,"text/plain","Removed. Rebooting...");
  delay(500);
  ESP.restart();
}

// ===================== SETUP/LOOP =====================
void setup() {
  Serial.begin(115200);
  delay(200);

  Serial.println("\nStarting APC8200 monitor on D1 mini (ESP8266)...");

  loadWifiCreds();
  connectWiFi();

  server.on("/", handleRoot);
  server.on("/data", handleData);
  server.on("/networks", handleNetworks);
  server.on("/add", handleAddNetwork);
  server.on("/remove", handleRemoveNetwork);
  server.begin();
  Serial.println("HTTP server started");

  SensorSer.begin(SENSOR_BAUD);
  Serial.println("Sensor SoftwareSerial started @19200");
}

void loop() {
  server.handleClient();
  MDNS.update();

  while (SensorSer.available()) {
    feedByte((uint8_t)SensorSer.read());
    yield();
  }
}
