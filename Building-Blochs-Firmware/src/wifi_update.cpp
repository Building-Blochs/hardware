#include "wifi_update.h"
#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <HTTPClient.h>
#include <Preferences.h>
#include <atomic>

namespace {
struct Credentials { char ssid[33]; char password[64]; } credentials{};
struct Job { bool configure; char url[1025]; Credentials credentials; };
QueueHandle_t jobs = nullptr, chunks = nullptr;
std::atomic<bool> busy{false}, cancelled{false};
std::atomic<uint8_t> state{0};

bool sendChunk(const WifiUpdateChunk &chunk) {
    while (!cancelled) {
        if (xQueueSend(chunks, &chunk, pdMS_TO_TICKS(100)) == pdTRUE) return true;
    }
    return false;
}
// Read the response with explicit idle deadlines. Some Arduino HTTPClient versions
// can wait forever for a stalled body, even when setTimeout() has been configured.
class BodyReader {
    WiFiClient &client;
public:
    explicit BodyReader(WiFiClient &client) : client(client) {}
    int read(uint8_t *data, size_t count) {
        uint32_t started = millis();
        while (!cancelled && millis() - started < 15000) {
            int available = client.available();
            if (available > 0) return client.read(data, std::min<size_t>(count, available));
            if (!client.connected()) return 0;
            vTaskDelay(pdMS_TO_TICKS(10));
        }
        return -1;
    }
    bool line(String &line) {
        line = "";
        uint32_t started = millis();
        while (line.length() < 128 && millis() - started < 15000) {
            uint8_t b;
            if (read(&b, 1) != 1) return false;
            if (b == '\n') return line.endsWith("\r");
            line += char(b);
        }
        return false;
    }
    bool transfer(size_t count) {
        while (count && !cancelled) {
            WifiUpdateChunk chunk{};
            chunk.size = read(chunk.bytes, std::min(count, sizeof(chunk.bytes)));
            if (chunk.size <= 0 || !sendChunk(chunk)) return false;
            count -= chunk.size;
        }
        return count == 0;
    }
};
constexpr size_t maxPackage = 0x640000 + 300;
bool readBody(HTTPClient &http) {
    BodyReader reader(http.getStream());
    String encoding = http.header("Transfer-Encoding");
    if (encoding.equalsIgnoreCase("chunked")) {
        size_t total = 0;
        for (;;) {
            String line;
            if (!reader.line(line) || !isxdigit(static_cast<unsigned char>(line[0]))) return false;
            char *end;
            unsigned long count = strtoul(line.c_str(), &end, 16);
            if ((*end != ';' && *end != '\r') || count > maxPackage - total) return false;
            if (!count) return true; // signed package validation checks the exact body length
            if (!reader.transfer(count) || !reader.line(line) || line != "\r") return false;
            total += count;
        }
    }
    if (encoding.length() && !encoding.equalsIgnoreCase("identity")) return false;
    int length = http.getSize();
    if (length >= 0) return length > 300 && size_t(length) <= maxPackage && reader.transfer(length);
    size_t total = 0;
    while (!cancelled) {
        WifiUpdateChunk chunk{};
        chunk.size = reader.read(chunk.bytes, sizeof(chunk.bytes));
        if (!chunk.size) return true; // connection-delimited response
        if (chunk.size < 0 || size_t(chunk.size) > maxPackage - total || !sendChunk(chunk)) return false;
        total += chunk.size;
    }
    return false;
}
void download(const char *url) {
    HTTPClient http;
    WiFiClient plain;
    WiFiClientSecure secure;
    // Trust the signed BBU, independent of the download host's certificate/clock.
    // Never send Wi-Fi credentials to this endpoint.
    secure.setInsecure();
    secure.setHandshakeTimeout(10);
    http.setConnectTimeout(5000);
    http.setTimeout(5000);
    http.setFollowRedirects(HTTPC_DISABLE_FOLLOW_REDIRECTS);
    bool https = strncmp(url, "https://", 8) == 0;
    bool ok = WiFi.status() == WL_CONNECTED &&
        http.begin(https ? static_cast<WiFiClient &>(secure) : plain, url);
    const char *headers[] = {"Transfer-Encoding"};
    if (ok) {
        http.collectHeaders(headers, 1);
        ok = http.GET() == HTTP_CODE_OK;
    }
    if (ok && !cancelled) {
        ok = readBody(http);
    }
    http.end();
    WifiUpdateChunk end{};
    end.size = ok ? 0 : -1;
    sendChunk(end);
}
void worker(void *) {
    WiFi.persistent(false);
    WiFi.mode(WIFI_STA);
    WiFi.setAutoReconnect(true);
    if (credentials.ssid[0]) {
        state = 1;
        WiFi.begin(credentials.ssid, credentials.password);
    }
    uint32_t lastProbe = millis() - 30000, lastConnect = millis();
    for (;;) {
        Job job{};
        if (xQueueReceive(jobs, &job, pdMS_TO_TICKS(100)) == pdTRUE) {
            if (job.configure) {
                credentials = job.credentials;
                WiFi.disconnect();
                state = 1;
                WiFi.begin(credentials.ssid, credentials.password);
                lastConnect = millis();
                lastProbe = millis() - 30000;
            } else {
                download(job.url);
            }
            memset(&job, 0, sizeof(job));
            busy = false;
        }
        if (WiFi.status() != WL_CONNECTED) {
            state = !credentials.ssid[0] ? 0 : (millis() - lastConnect < 20000 ? 1 : 4);
            if (credentials.ssid[0] && millis() - lastConnect > 30000) {
                WiFi.reconnect();
                lastConnect = millis();
            }
            continue;
        }
        if (state != 3) state = 2;
        if (millis() - lastProbe >= 30000 && !busy) {
            HTTPClient probe;
            WiFiClient client;
            probe.setConnectTimeout(3000);
            probe.setTimeout(3000);
            bool online = probe.begin(client, "http://connectivitycheck.gstatic.com/generate_204") && probe.GET() == 204;
            probe.end();
            state = online ? 3 : 2;
            lastProbe = millis();
        }
    }
}
}
void wifi_update_setup() {
    Preferences prefs;
    if (prefs.begin("wifi", true)) {
        if (prefs.getBytesLength("credentials") == sizeof(credentials))
            prefs.getBytes("credentials", &credentials, sizeof(credentials));
        prefs.end();
    }
    credentials.ssid[32] = 0;
    credentials.password[63] = 0;
    jobs = xQueueCreate(1, sizeof(Job));
    chunks = xQueueCreate(4, sizeof(WifiUpdateChunk));
    if (!jobs || !chunks || xTaskCreate(worker, "wifi_update", 8192, nullptr, 1, nullptr) != pdPASS) {
        busy = true; // unavailable: do not accept requests without a worker
        state = 4;
    }
}
bool wifi_update_busy() { return busy.load(); }
uint8_t wifi_update_state() { return state.load(); }
bool wifi_update_configure(const uint8_t *data, size_t size) {
    if (busy || size < 2 || data[0] < 1 || data[0] > 32 || size < size_t(1 + data[0])) return false;
    size_t passwordSize = size - 1 - data[0];
    if (passwordSize > 63 || (passwordSize && passwordSize < 8) || memchr(data + 1, 0, size - 1)) return false;
    Job job{};
    job.configure = true;
    memcpy(job.credentials.ssid, data + 1, data[0]);
    memcpy(job.credentials.password, data + 1 + data[0], passwordSize);
    Preferences prefs;
    if (!prefs.begin("wifi", false)) return false;
    bool saved = prefs.putBytes("credentials", &job.credentials, sizeof(job.credentials)) == sizeof(job.credentials);
    prefs.end();
    if (!saved) return false;
    busy = true;
    state = 1;
    if (xQueueSend(jobs, &job, 0) != pdTRUE) { busy = false; return false; }
    memset(&job, 0, sizeof(job));
    return true;
}
bool wifi_update_start(const char *url) {
    if (busy || WiFi.status() != WL_CONNECTED || strlen(url) > 1024) return false;
    Job job{};
    strncpy(job.url, url, sizeof(job.url) - 1);
    xQueueReset(chunks);
    cancelled = false;
    busy = true;
    if (xQueueSend(jobs, &job, 0) != pdTRUE) { busy = false; return false; }
    return true;
}
void wifi_update_cancel() { cancelled = true; }
bool wifi_update_read(WifiUpdateChunk &chunk) { return chunks && xQueueReceive(chunks, &chunk, 0) == pdTRUE; }
