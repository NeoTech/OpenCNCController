/**
 * @file eth_comm.c
 * @brief Ethernet Communication Implementation
 */

#include "eth_comm.h"
#include "canopen_master.h"
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_eth.h"
#include "esp_netif.h"
#include "esp_event.h"
#include "lwip/sockets.h"
#include "lwip/netdb.h"

static const char *TAG = "eth_comm";

// Global state
static eth_state_t g_eth = {0};
static char g_ip_str[16] = "0.0.0.0";

// Forward declarations
static void eth_event_handler(void* arg, esp_event_base_t event_base,
                             int32_t event_id, void* event_data);
static void ip_event_handler(void* arg, esp_event_base_t event_base,
                            int32_t event_id, void* event_data);
static void process_packet(const uint8_t* data, uint16_t length);
static void send_response(uint8_t status, const uint8_t* payload, uint16_t length);
static uint16_t crc16(const uint8_t* data, uint16_t length);

// =============================================================================
// Initialization
// =============================================================================

void eth_comm_init(void)
{
    ESP_LOGI(TAG, "Initializing Ethernet");
    
    memset(&g_eth, 0, sizeof(g_eth));
    g_eth.socket = -1;
    
    // Initialize TCP/IP stack
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    
    // Create default Ethernet netif
    esp_netif_config_t netif_cfg = ESP_NETIF_DEFAULT_ETH();
    esp_netif_t* eth_netif = esp_netif_new(&netif_cfg);
    
    // Configure Ethernet MAC and PHY
    eth_mac_config_t mac_config = ETH_MAC_DEFAULT_CONFIG();
    eth_phy_config_t phy_config = ETH_PHY_DEFAULT_CONFIG();
    
    // ESP32-P4 internal EMAC
    eth_esp32_emac_config_t esp32_emac_config = ETH_ESP32_EMAC_DEFAULT_CONFIG();
    esp_eth_mac_t* mac = esp_eth_mac_new_esp32(&esp32_emac_config, &mac_config);
    
    // LAN8720 PHY (typical for ESP32 dev boards)
    esp_eth_phy_t* phy = esp_eth_phy_new_lan87xx(&phy_config);
    
    // Create Ethernet driver
    esp_eth_config_t eth_config = ETH_DEFAULT_CONFIG(mac, phy);
    esp_eth_handle_t eth_handle = NULL;
    ESP_ERROR_CHECK(esp_eth_driver_install(&eth_config, &eth_handle));
    
    // Attach driver to netif
    ESP_ERROR_CHECK(esp_netif_attach(eth_netif, esp_eth_new_netif_glue(eth_handle)));
    
    // Register event handlers
    ESP_ERROR_CHECK(esp_event_handler_register(ETH_EVENT, ESP_EVENT_ANY_ID, 
                                                &eth_event_handler, NULL));
    ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, IP_EVENT_ETH_GOT_IP,
                                                &ip_event_handler, NULL));
    
    // Start Ethernet
    ESP_ERROR_CHECK(esp_eth_start(eth_handle));
    
    g_eth.initialized = true;
    ESP_LOGI(TAG, "Ethernet initialized");
}

// =============================================================================
// Event Handlers
// =============================================================================

static void eth_event_handler(void* arg, esp_event_base_t event_base,
                             int32_t event_id, void* event_data)
{
    switch (event_id) {
        case ETHERNET_EVENT_CONNECTED:
            ESP_LOGI(TAG, "Ethernet link up");
            break;
        case ETHERNET_EVENT_DISCONNECTED:
            ESP_LOGI(TAG, "Ethernet link down");
            g_eth.connected = false;
            g_eth.got_ip = false;
            if (g_eth.socket >= 0) {
                close(g_eth.socket);
                g_eth.socket = -1;
            }
            break;
        case ETHERNET_EVENT_START:
            ESP_LOGI(TAG, "Ethernet started");
            break;
        case ETHERNET_EVENT_STOP:
            ESP_LOGI(TAG, "Ethernet stopped");
            break;
        default:
            break;
    }
}

static void ip_event_handler(void* arg, esp_event_base_t event_base,
                            int32_t event_id, void* event_data)
{
    ip_event_got_ip_t* event = (ip_event_got_ip_t*)event_data;
    
    snprintf(g_ip_str, sizeof(g_ip_str), IPSTR, IP2STR(&event->ip_info.ip));
    ESP_LOGI(TAG, "Got IP: %s", g_ip_str);
    
    g_eth.got_ip = true;
}

// =============================================================================
// TCP Server
// =============================================================================

static void start_server(void)
{
    struct sockaddr_in server_addr = {
        .sin_family = AF_INET,
        .sin_port = htons(CONFIG_OPENCNC_HMI_PORT),
        .sin_addr.s_addr = htonl(INADDR_ANY)
    };
    
    int listen_sock = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
    if (listen_sock < 0) {
        ESP_LOGE(TAG, "Failed to create socket");
        return;
    }
    
    int opt = 1;
    setsockopt(listen_sock, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));
    
    if (bind(listen_sock, (struct sockaddr*)&server_addr, sizeof(server_addr)) < 0) {
        ESP_LOGE(TAG, "Failed to bind socket");
        close(listen_sock);
        return;
    }
    
    if (listen(listen_sock, 1) < 0) {
        ESP_LOGE(TAG, "Failed to listen");
        close(listen_sock);
        return;
    }
    
    ESP_LOGI(TAG, "Listening on port %d", CONFIG_OPENCNC_HMI_PORT);
    
    // Accept connection (blocking with timeout)
    struct timeval timeout = { .tv_sec = 1, .tv_usec = 0 };
    setsockopt(listen_sock, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof(timeout));
    
    struct sockaddr_in client_addr;
    socklen_t addr_len = sizeof(client_addr);
    
    g_eth.socket = accept(listen_sock, (struct sockaddr*)&client_addr, &addr_len);
    if (g_eth.socket >= 0) {
        ESP_LOGI(TAG, "Client connected from " IPSTR, IP2STR(&client_addr.sin_addr));
        g_eth.connected = true;
        g_eth.last_rx_tick = xTaskGetTickCount();
        
        // Set non-blocking
        fcntl(g_eth.socket, F_SETFL, O_NONBLOCK);
    }
    
    close(listen_sock);
}

// =============================================================================
// Tick
// =============================================================================

void eth_comm_tick(void)
{
    if (!g_eth.initialized) return;
    
    // Wait for IP address
    if (!g_eth.got_ip) return;
    
    // Start server if no connection
    if (g_eth.socket < 0) {
        start_server();
        return;
    }
    
    // Check for connection timeout
    if (xTaskGetTickCount() - g_eth.last_rx_tick > pdMS_TO_TICKS(30000)) {
        ESP_LOGW(TAG, "Connection timeout");
        eth_comm_disconnect();
        return;
    }
    
    // Read incoming data
    int len = recv(g_eth.socket, g_eth.rx_buffer + g_eth.rx_pos, 
                   sizeof(g_eth.rx_buffer) - g_eth.rx_pos, 0);
    
    if (len > 0) {
        g_eth.rx_pos += len;
        g_eth.last_rx_tick = xTaskGetTickCount();
        
        // Process complete packets
        while (g_eth.rx_pos >= ETH_HEADER_SIZE) {
            eth_header_t* hdr = (eth_header_t*)g_eth.rx_buffer;
            
            // Check magic
            if (hdr->magic != ETH_MAGIC_REQUEST) {
                // Invalid data, skip byte
                memmove(g_eth.rx_buffer, g_eth.rx_buffer + 1, g_eth.rx_pos - 1);
                g_eth.rx_pos--;
                continue;
            }
            
            // Check if complete packet received
            uint16_t total_len = ETH_HEADER_SIZE + hdr->length;
            if (g_eth.rx_pos < total_len) break;
            
            // Verify CRC
            uint16_t calc_crc = crc16(g_eth.rx_buffer + ETH_HEADER_SIZE, hdr->length);
            if (calc_crc != hdr->crc) {
                ESP_LOGW(TAG, "CRC mismatch");
                send_response(RESP_ERROR_CRC_MISMATCH, NULL, 0);
                g_eth.error_count++;
            } else {
                // Process packet
                g_eth.rx_count++;
                process_packet(g_eth.rx_buffer, total_len);
            }
            
            // Remove processed packet
            memmove(g_eth.rx_buffer, g_eth.rx_buffer + total_len, g_eth.rx_pos - total_len);
            g_eth.rx_pos -= total_len;
        }
    } else if (len == 0) {
        // Connection closed
        ESP_LOGI(TAG, "Client disconnected");
        eth_comm_disconnect();
    } else if (errno != EAGAIN && errno != EWOULDBLOCK) {
        // Error
        ESP_LOGE(TAG, "Socket error: %d", errno);
        eth_comm_disconnect();
    }
}

// =============================================================================
// Packet Processing
// =============================================================================

static void process_packet(const uint8_t* data, uint16_t length)
{
    eth_header_t* hdr = (eth_header_t*)data;
    const uint8_t* payload = data + ETH_HEADER_SIZE;
    
    switch (hdr->command) {
        case CMD_PING:
            send_response(RESP_OK, NULL, 0);
            break;
            
        case CMD_GET_VERSION: {
            uint8_t version[] = { 1, 0, 0, ETH_PROTOCOL_VERSION };
            send_response(RESP_OK, version, sizeof(version));
            break;
        }
        
        case CMD_GET_STATUS: {
            eth_status_payload_t status = {0};
            // TODO: Fill from system state
            status.axis_count = canopen_master_get_axis_count();
            status.io_node_count = canopen_master_get_io_node_count();
            
            for (int i = 0; i < status.axis_count && i < 9; i++) {
                status.position[i] = canopen_master_get_position(i + 1);
                status.statusword[i] = canopen_master_get_statusword(i + 1);
            }
            
            send_response(RESP_OK, (uint8_t*)&status, sizeof(status));
            break;
        }
        
        case CMD_GET_NODE_COUNT: {
            uint8_t counts[] = { 
                canopen_master_get_axis_count(),
                canopen_master_get_io_node_count()
            };
            send_response(RESP_OK, counts, sizeof(counts));
            break;
        }
        
        case CMD_GET_NODE_CONFIG: {
            if (hdr->length < 1) {
                send_response(RESP_ERROR_INVALID_PARAM, NULL, 0);
                break;
            }
            uint8_t node_id = payload[0];
            const node_config_t* cfg = canopen_master_get_node_config(node_id);
            if (cfg) {
                eth_node_config_payload_t resp = {
                    .node_id = cfg->node_id,
                    .type = cfg->type,
                    .axis = cfg->axis,
                    .capabilities_lo = cfg->capabilities & 0xFF,
                    .capabilities_hi = (cfg->capabilities >> 8) & 0xFF,
                    .digital_in_count = cfg->digital_inputs,
                    .digital_out_count = cfg->digital_outputs,
                    .max_velocity = cfg->max_velocity,
                    .max_acceleration = cfg->max_acceleration,
                    .steps_per_unit = cfg->steps_per_unit
                };
                send_response(RESP_OK, (uint8_t*)&resp, sizeof(resp));
            } else {
                send_response(RESP_ERROR_INVALID_PARAM, NULL, 0);
            }
            break;
        }
        
        case CMD_ENABLE_AXIS: {
            if (hdr->length < 1) {
                send_response(RESP_ERROR_INVALID_PARAM, NULL, 0);
                break;
            }
            canopen_master_enable_axis(payload[0]);
            send_response(RESP_OK, NULL, 0);
            break;
        }
        
        case CMD_DISABLE_AXIS: {
            if (hdr->length < 1) {
                send_response(RESP_ERROR_INVALID_PARAM, NULL, 0);
                break;
            }
            canopen_master_disable_axis(payload[0]);
            send_response(RESP_OK, NULL, 0);
            break;
        }
        
        case CMD_HOME_AXIS: {
            if (hdr->length < 1) {
                send_response(RESP_ERROR_INVALID_PARAM, NULL, 0);
                break;
            }
            canopen_master_start_homing(payload[0], HOMING_SWITCH_NEGATIVE);
            send_response(RESP_OK, NULL, 0);
            break;
        }
        
        case CMD_HOME_ALL:
            canopen_master_home_all_axes();
            send_response(RESP_OK, NULL, 0);
            break;
            
        case CMD_ESTOP:
            canopen_master_stop_all_nodes();
            send_response(RESP_OK, NULL, 0);
            break;
            
        case CMD_GET_POSITION: {
            if (hdr->length < 1) {
                send_response(RESP_ERROR_INVALID_PARAM, NULL, 0);
                break;
            }
            int32_t pos = canopen_master_get_position(payload[0]);
            send_response(RESP_OK, (uint8_t*)&pos, sizeof(pos));
            break;
        }
        
        case CMD_SET_POSITION: {
            if (hdr->length < 5) {
                send_response(RESP_ERROR_INVALID_PARAM, NULL, 0);
                break;
            }
            uint8_t axis = payload[0];
            int32_t pos;
            memcpy(&pos, payload + 1, sizeof(pos));
            canopen_master_set_position(axis, pos);
            send_response(RESP_OK, NULL, 0);
            break;
        }
        
        default:
            ESP_LOGW(TAG, "Unknown command: 0x%02X", hdr->command);
            send_response(RESP_ERROR_UNKNOWN_CMD, NULL, 0);
            break;
    }
}

// =============================================================================
// Response Sending
// =============================================================================

static void send_response(uint8_t status, const uint8_t* payload, uint16_t length)
{
    if (g_eth.socket < 0) return;
    
    eth_response_header_t* hdr = (eth_response_header_t*)g_eth.tx_buffer;
    hdr->magic = ETH_MAGIC_RESPONSE;
    hdr->status = status;
    hdr->length = length;
    hdr->crc = crc16(payload, length);
    
    if (payload && length > 0) {
        memcpy(g_eth.tx_buffer + sizeof(eth_response_header_t), payload, length);
    }
    
    int total = sizeof(eth_response_header_t) + length;
    int sent = send(g_eth.socket, g_eth.tx_buffer, total, 0);
    
    if (sent == total) {
        g_eth.tx_count++;
    } else {
        g_eth.error_count++;
    }
}

// =============================================================================
// Utility Functions
// =============================================================================

static uint16_t crc16(const uint8_t* data, uint16_t length)
{
    uint16_t crc = 0xFFFF;
    
    for (uint16_t i = 0; i < length; i++) {
        crc ^= data[i];
        for (int j = 0; j < 8; j++) {
            if (crc & 1) {
                crc = (crc >> 1) ^ 0xA001;
            } else {
                crc >>= 1;
            }
        }
    }
    
    return crc;
}

bool eth_comm_is_connected(void)
{
    return g_eth.connected;
}

void eth_comm_get_stats(uint32_t* rx_count, uint32_t* tx_count, uint32_t* errors)
{
    if (rx_count) *rx_count = g_eth.rx_count;
    if (tx_count) *tx_count = g_eth.tx_count;
    if (errors) *errors = g_eth.error_count;
}

void eth_comm_disconnect(void)
{
    if (g_eth.socket >= 0) {
        close(g_eth.socket);
        g_eth.socket = -1;
    }
    g_eth.connected = false;
    g_eth.rx_pos = 0;
}

const char* eth_comm_get_ip_address(void)
{
    return g_ip_str;
}

void eth_comm_send_status_update(void)
{
    // TODO: Implement unsolicited status push
}

void eth_comm_send_alarm(uint8_t node_id, uint32_t alarm_code)
{
    // TODO: Implement alarm notification
}
