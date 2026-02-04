# 🏗️ Arquitectura y Funcionamiento Completo del Sistema

## 📚 Tabla de Contenidos

1. [Visión General](#visión-general)
2. [¿Qué es ROS 2?](#qué-es-ros-2)
3. [¿Qué es micro-ROS?](#qué-es-micro-ros)
4. [Arquitectura del Sistema](#arquitectura-del-sistema)
5. [Flujo de Comunicación Completo](#flujo-de-comunicación-completo)
6. [Análisis del Código ESP32](#análisis-del-código-esp32)
7. [Análisis del Código Python](#análisis-del-código-python)
8. [Protocolo de Comunicación](#protocolo-de-comunicación)
9. [Conceptos Clave](#conceptos-clave)

---

## 🌟 Visión General

Este proyecto implementa un sistema de comunicación entre:
- **Computadora** → Publicador (envía comandos)
- **Agente micro-ROS** → Intermediario (traduce mensajes)
- **ESP32** → Suscriptor (recibe comandos y controla LED)

```
┌─────────────┐      ┌──────────────┐      ┌──────────┐
│ Computadora │─WiFi─│ Agente       │─UDP──│  ESP32   │
│ (ROS 2)     │      │ micro-ROS    │      │(micro-ROS)│
└─────────────┘      └──────────────┘      └──────────┘
     Python              C/C++                  C
```

---

## 🤖 ¿Qué es ROS 2?

### Robot Operating System 2

**ROS 2** NO es un sistema operativo, es un **framework de middleware** para robots y sistemas distribuidos.

### Características Principales:

1. **Sistema de Mensajería Distribuida**
   - Permite que diferentes programas se comuniquen entre sí
   - No importa el lenguaje (Python, C++, Rust, etc.)
   - No importa dónde están (misma PC, red local, internet)

2. **Arquitectura de Nodos**
   ```
   Nodo = Programa independiente que hace UNA tarea
   ```
   - Un nodo para cámara
   - Un nodo para control de motor
   - Un nodo para procesamiento de imagen
   - etc.

3. **Tópicos (Topics)**
   ```
   Tópico = Canal de comunicación con nombre
   ```
   - Los nodos publican mensajes en tópicos
   - Los nodos se suscriben a tópicos para recibir mensajes
   - Comunicación asíncrona (publicador y suscriptor no necesitan conocerse)

4. **Protocolo DDS** (Data Distribution Service)
   - Estándar industrial para comunicación en tiempo real
   - Auto-descubrimiento (nodos se encuentran automáticamente)
   - QoS (Quality of Service) configurable

### Ejemplo Conceptual:

```python
# Nodo Publicador (keyboard_led_control.py)
publicador = crea_publicador('/led_control')
publicador.publica("ON")  # Envía mensaje

# Nodo Suscriptor (ESP32)
def callback(mensaje):
    print(f"Recibí: {mensaje}")
    
suscriptor = crea_suscriptor('/led_control', callback)
# Cuando llegue mensaje → se ejecuta callback()
```

---

## 🔬 ¿Qué es micro-ROS?

### ROS 2 para Microcontroladores

**micro-ROS** es una **versión reducida de ROS 2** diseñada para dispositivos con recursos limitados como el ESP32.

### ¿Por qué no usar ROS 2 normal en ESP32?

| Requisito | ROS 2 | micro-ROS | ESP32 |
|-----------|-------|-----------|-------|
| RAM | ~500MB | ~50KB | 520KB ✅ |
| CPU | Multi-core GHz | Single 240MHz | 240MHz ✅ |
| OS | Linux/Windows | FreeRTOS | FreeRTOS ✅ |
| DDS | Full | XRCE-DDS | XRCE-DDS ✅ |

### Diferencias Clave:

**ROS 2 (Computadora):**
```
┌──────────────────────────┐
│  Aplicación (Python/C++) │
├──────────────────────────┤
│  RCL (ROS Client Library)│
├──────────────────────────┤
│  DDS (FastDDS, CycloneDDS)│ ← Protocolo completo
├──────────────────────────┤
│  Red (UDP/TCP)           │
└──────────────────────────┘
```

**micro-ROS (ESP32):**
```
┌──────────────────────────┐
│  Aplicación (C)          │
├──────────────────────────┤
│  RCLC (Client Library C) │ ← Versión reducida
├──────────────────────────┤
│  XRCE-DDS Client         │ ← Protocolo ligero
├──────────────────────────┤
│  Red (UDP)               │
└──────────────────────────┘
```

### El Agente micro-ROS: El Traductor

```
ESP32 (XRCE-DDS) ←→ Agente ←→ ROS 2 (DDS)
```

El **agente** es el intermediario que:
1. Habla XRCE-DDS con el ESP32
2. Habla DDS con ROS 2
3. Traduce entre ambos protocolos

---

## 🏛️ Arquitectura del Sistema

### Vista de Alto Nivel

```
                    RED WiFi 192.168.0.x
    ┌─────────────────────────────────────────────┐
    │                                             │
┌───┴──────────┐      ┌──────────────┐      ┌────┴─────┐
│  Computadora │      │    Agente    │      │  ESP32   │
│192.168.0.76  │      │micro-ROS     │      │192.168.0 │
│              │      │(en PC)       │      │   .69    │
└──────────────┘      └──────────────┘      └──────────┘
     ROS 2           Puerto 8888 UDP          micro-ROS
```

### Componentes Detallados

#### 1. Script Python (keyboard_led_control.py)
```python
┌─────────────────────────────────────┐
│  KeyboardLEDController              │
├─────────────────────────────────────┤
│  • Lee teclas del usuario           │
│  • Crea mensajes String             │
│  • Publica en /led_control          │
└─────────────────────────────────────┘
         │
         │ String("ON")
         ▼
┌─────────────────────────────────────┐
│  ROS 2 DDS (FastDDS/CycloneDDS)     │
│  • Auto-descubrimiento              │
│  • Serializa mensaje                │
│  • Envía por red                    │
└─────────────────────────────────────┘
```

#### 2. Agente micro-ROS
```
┌────────────────────────────────────────┐
│  micro_ros_agent                       │
├────────────────────────────────────────┤
│  • Escucha puerto UDP 8888             │
│  • Mantiene sesión con ESP32           │
│  • Traduce XRCE-DDS ↔ DDS             │
│  • Gestiona QoS y reliability          │
└────────────────────────────────────────┘
         ▲                    │
    XRCE-DDS              DDS
         │                    ▼
    ESP32              ROS 2 Network
```

#### 3. Firmware ESP32 (main.c)
```c
┌────────────────────────────────────────┐
│  FreeRTOS Task                         │
├────────────────────────────────────────┤
│  1. WiFi Task                          │
│     └→ Conecta a red                   │
│  2. micro-ROS Task                     │
│     ├→ Conecta al agente               │
│     ├→ Crea nodo: /led_controller      │
│     ├→ Suscribe a: /led_control        │
│     └→ Spin executor (loop infinito)   │
│  3. Callback Function                  │
│     └→ Procesa mensaje y controla GPIO │
└────────────────────────────────────────┘
```

---

## 🔄 Flujo de Comunicación Completo

### Paso a Paso: "Usuario presiona tecla E (Encender LED)"

#### FASE 1: Input del Usuario
```
┌────────────────────────────────┐
│ 1. Usuario presiona tecla "E"  │
└────────────────────────────────┘
                │
                ▼
┌────────────────────────────────────────────┐
│ 2. keyboard_led_control.py detecta tecla   │
│    • get_key() lee entrada                 │
│    • if key == 'e': comando = "ON"         │
└────────────────────────────────────────────┘
                │
                ▼
┌─────────────────────────────────────────────┐
│ 3. Crea mensaje String                      │
│    msg = String()                           │
│    msg.data = "ON"                          │
└─────────────────────────────────────────────┘
```

#### FASE 2: Publicación ROS 2
```
┌────────────────────────────────────────────┐
│ 4. Publica en tópico /led_control          │
│    publisher.publish(msg)                  │
└────────────────────────────────────────────┘
                │
                ▼
┌────────────────────────────────────────────┐
│ 5. ROS 2 DDS serializa mensaje             │
│    • Convierte String a bytes              │
│    • Añade metadata (timestamp, QoS)      │
│    • Tipo: std_msgs/msg/String            │
└────────────────────────────────────────────┘
                │
                ▼
┌────────────────────────────────────────────┐
│ 6. DDS busca suscriptores                  │
│    • Auto-descubrimiento de nodos          │
│    • Encuentra: agente micro-ROS           │
└────────────────────────────────────────────┘
```

#### FASE 3: Agente micro-ROS
```
┌────────────────────────────────────────────┐
│ 7. Agente recibe mensaje DDS               │
│    • Topic: /led_control                   │
│    • Data: "ON"                            │
└────────────────────────────────────────────┘
                │
                ▼
┌────────────────────────────────────────────┐
│ 8. Agente busca cliente XRCE-DDS           │
│    • Sesión activa: 0x81                   │
│    • Cliente: ESP32 (0x2C4FECFA)           │
│    • IP: 192.168.0.69                      │
└────────────────────────────────────────────┘
                │
                ▼
┌────────────────────────────────────────────┐
│ 9. Traduce DDS → XRCE-DDS                  │
│    • Serializa en formato XRCE             │
│    • Mensaje más compacto                  │
│    • Menos overhead                        │
└────────────────────────────────────────────┘
                │
                ▼
┌────────────────────────────────────────────┐
│ 10. Envía por UDP                          │
│     • Destino: 192.168.0.69:xxxxx         │
│     • Puerto: dinámico (asignado)          │
│     • Protocolo: UDP                       │
└────────────────────────────────────────────┘
```

#### FASE 4: ESP32 Recibe
```
┌────────────────────────────────────────────┐
│ 11. ESP32 recibe paquete UDP               │
│     • Stack TCP/IP (lwIP)                  │
│     • Buffer de recepción                  │
└────────────────────────────────────────────┘
                │
                ▼
┌────────────────────────────────────────────┐
│ 12. XRCE-DDS Client deserializa            │
│     • Lee tipo de mensaje                  │
│     • Extrae datos: "ON"                   │
└────────────────────────────────────────────┘
                │
                ▼
┌────────────────────────────────────────────┐
│ 13. rclc_executor procesa mensaje          │
│     • Identifica suscriptor                │
│     • Topic: /led_control                  │
│     • Llama callback registrado            │
└────────────────────────────────────────────┘
```

#### FASE 5: Ejecución en ESP32
```
┌────────────────────────────────────────────┐
│ 14. subscription_callback() se ejecuta     │
│     • Parámetro: msgin (puntero al mensaje)│
└────────────────────────────────────────────┘
                │
                ▼
┌────────────────────────────────────────────┐
│ 15. Procesa comando                        │
│     const char* data = msg->data.data;     │
│     if (strcmp(data, "ON") == 0) {         │
│         led_on();                          │
│     }                                      │
└────────────────────────────────────────────┘
                │
                ▼
┌────────────────────────────────────────────┐
│ 16. led_on() ejecuta                       │
│     • gpio_set_level(LED_GPIO, 1)          │
│     • Pin GPIO 2 → 3.3V                    │
│     • LED enciende ✅                      │
└────────────────────────────────────────────┘
                │
                ▼
┌────────────────────────────────────────────┐
│ 17. Log en consola                         │
│     ESP_LOGI("💡 LED encendido")           │
└────────────────────────────────────────────┘
```

### Tiempo Total: ~50-100ms

```
Evento          Tiempo    Acumulado
────────────────────────────────────
Presionar tecla    1ms        1ms
Python detecta     5ms        6ms
ROS 2 publica      2ms        8ms
DDS procesa        3ms       11ms
Agente traduce     2ms       13ms
UDP envía          5ms       18ms
ESP32 recibe      10ms       28ms
Callback ejecuta   1ms       29ms
GPIO cambia      <1ms       30ms
LED responde     10ms       40ms ✅
```

---

## 📝 Análisis del Código ESP32

Vamos línea por línea del archivo `main.c`:

### 1. Includes y Configuración

```c
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <std_msgs/msg/string.h>
```

- `rcl/rcl.h`: ROS Client Library (funciones básicas)
- `rclc/rclc.h`: ROS Client Library C (API simplificada para C)
- `std_msgs/msg/string.h`: Definición del tipo String

```c
#define LED_GPIO                2       
#define AGENT_IP                CONFIG_MICRO_ROS_AGENT_IP
#define AGENT_PORT              CONFIG_MICRO_ROS_AGENT_PORT
```

**¿Qué hace?** Define constantes del proyecto.

### 2. Inicialización WiFi

```c
static void wifi_init(void)
{
    esp_netif_init();                           // Inicia stack TCP/IP
    esp_event_loop_create_default();           // Sistema de eventos
    esp_netif_create_default_wifi_sta();       // Modo Station (cliente)
    
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    esp_wifi_init(&cfg);                        // Inicializa driver WiFi
    
    esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID, 
                                &wifi_event_handler, NULL);
    
    // Configuración de red
    wifi_config_t wifi_config = {
        .sta = {
            .ssid = WIFI_SSID,
            .password = WIFI_PASS,
        },
    };
    
    esp_wifi_set_mode(WIFI_MODE_STA);          // Modo estación
    esp_wifi_set_config(WIFI_IF_STA, &wifi_config);
    esp_wifi_start();                           // Inicia WiFi
    esp_wifi_connect();                         // Conecta a AP
}
```

**¿Qué hace?**
1. Inicializa el stack TCP/IP de ESP32 (lwIP)
2. Crea una interfaz WiFi en modo estación (cliente)
3. Configura SSID y contraseña
4. Se conecta al router WiFi
5. Obtiene IP por DHCP (192.168.0.69)

### 3. Funciones de Control del LED

```c
static void led_init(void)
{
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << LED_GPIO),    // Máscara para GPIO 2
        .mode = GPIO_MODE_OUTPUT,               // Modo salida
        .pull_up_en = GPIO_PULLUP_ENABLE,      // Pull-up interno
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,        // Sin interrupciones
    };
    gpio_config(&io_conf);
    gpio_set_level(LED_GPIO, 0);               // Apagado inicial
}
```

**¿Qué hace?**
- Configura GPIO 2 como salida digital
- Activa resistencia pull-up interna
- Establece nivel inicial en 0 (LED apagado)

```c
static void led_on(void)
{
    gpio_set_level(LED_GPIO, 1);               // GPIO 2 → 3.3V
    ESP_LOGI(TAG, "💡 LED encendido");
}

static void led_off(void)
{
    gpio_set_level(LED_GPIO, 0);               // GPIO 2 → 0V
    ESP_LOGI(TAG, "💡 LED apagado");
}

static void led_toggle(void)
{
    static bool state = false;
    state = !state;
    gpio_set_level(LED_GPIO, state);
    ESP_LOGI(TAG, "💡 LED alternado: %s", state ? "ON" : "OFF");
}
```

**¿Qué hace?**
- `led_on()`: Pone GPIO en HIGH (3.3V) → enciende LED
- `led_off()`: Pone GPIO en LOW (0V) → apaga LED
- `led_toggle()`: Cambia de estado (variable estática mantiene estado)

### 4. Callback del Suscriptor

```c
static void subscription_callback(const void *msgin)
{
    // Cast del mensaje genérico a String
    const std_msgs__msg__String *msg = (const std_msgs__msg__String *)msgin;
    
    ESP_LOGI(TAG, "📨 Mensaje recibido: '%s'", msg->data.data);
    
    // Comparación de strings
    if (strcmp(msg->data.data, "ON") == 0 || 
        strcmp(msg->data.data, "1") == 0) {
        led_on();
    }
    else if (strcmp(msg->data.data, "OFF") == 0 || 
             strcmp(msg->data.data, "0") == 0) {
        led_off();
    }
    else if (strcmp(msg->data.data, "TOGGLE") == 0) {
        led_toggle();
    }
    else {
        ESP_LOGW(TAG, "⚠️  Comando desconocido: '%s'", msg->data.data);
    }
}
```

**¿Qué hace?**
1. **Recibe puntero genérico** (`void*`) al mensaje
2. **Hace cast** a tipo específico (`std_msgs__msg__String*`)
3. **Accede a los datos** con `msg->data.data` (string C)
4. **Compara comandos** con `strcmp()`
5. **Ejecuta función** correspondiente

**Estructura del mensaje String:**
```c
typedef struct std_msgs__msg__String {
    rosidl_runtime_c__String data;  // Tipo String de ROS
} std_msgs__msg__String;

typedef struct rosidl_runtime_c__String {
    char* data;        // Puntero al string
    size_t size;       // Tamaño actual
    size_t capacity;   // Capacidad del buffer
} rosidl_runtime_c__String;
```

### 5. Tarea Principal micro-ROS

```c
void micro_ros_task(void *arg)
{
    // 1. INICIALIZACIÓN
    rcl_allocator_t allocator = rcl_get_default_allocator();
    rclc_support_t support;
    
    rcl_init_options_t init_options = rcl_get_zero_initialized_init_options();
    RCCHECK(rcl_init_options_init(&init_options, allocator));
```

**¿Qué hace?**
- Obtiene el allocator de memoria por defecto
- Inicializa opciones de RCL (ROS Client Library)

```c
    // 2. CONFIGURAR TRANSPORTE UDP
#ifdef CONFIG_MICRO_ROS_ESP_XRCE_DDS_MIDDLEWARE
    rmw_init_options_t* rmw_options = rcl_init_options_get_rmw_init_options(&init_options);
    RCCHECK(rmw_uros_options_set_udp_address(AGENT_IP, AGENT_PORT, rmw_options));
#endif
```

**¿Qué hace?**
- Obtiene opciones de RMW (ROS MiddleWare)
- **CRÍTICO**: Configura IP y puerto del agente en las opciones
- Sin esto, el ESP32 no sabe dónde está el agente

```c
    // 3. PING AL AGENTE
#ifdef CONFIG_MICRO_ROS_ESP_XRCE_DDS_MIDDLEWARE
    while (rmw_uros_ping_agent_options(PING_TIMEOUT_MS, 1, rmw_options) != RMW_RET_OK) {
        ESP_LOGW(TAG, "Esperando agente...");
        vTaskDelay(pdMS_TO_TICKS(RECONNECT_DELAY_MS));
    }
#endif
    ESP_LOGI(TAG, "✅ Agente detectado");
```

**¿Qué hace?**
- **Intenta hacer ping** al agente usando las opciones configuradas
- Si falla, espera 2 segundos y reintenta
- Bucle infinito hasta que el agente responde
- **Usa `rmw_options`** para saber dónde hacer ping

```c
    // 4. CREAR SOPORTE Y NODO
    RCCHECK(rclc_support_init_with_options(&support, 0, NULL, &init_options, &allocator));
    
    rcl_node_t node = rcl_get_zero_initialized_node();
    RCCHECK(rclc_node_init_default(&node, "led_controller", "", &support));
    ESP_LOGI(TAG, "✅ Nodo 'led_controller' creado");
```

**¿Qué hace?**
- Inicializa el soporte de micro-ROS con las opciones configuradas
- Crea un nodo con nombre "led_controller"
- El nodo es visible en ROS 2 como `/led_controller`

```c
    // 5. CREAR SUSCRIPTOR
    RCCHECK(rclc_subscription_init_default(
        &subscriber,                              // Variable global
        &node,                                    // Nodo padre
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String),  // Tipo
        "/led_control"));                         // Nombre del tópico
    ESP_LOGI(TAG, "✅ Suscrito a /led_control");
```

**¿Qué hace?**
- Crea un suscriptor al tópico `/led_control`
- Tipo de mensaje: `std_msgs/msg/String`
- `ROSIDL_GET_MSG_TYPE_SUPPORT`: Macro que obtiene metadata del tipo

```c
    // 6. CREAR EXECUTOR
    rclc_executor_t executor = rclc_executor_get_zero_initialized_executor();
    RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));
    RCCHECK(rclc_executor_add_subscription(
        &executor,           // Executor
        &subscriber,         // Suscriptor a añadir
        &incoming_msg,       // Buffer para mensaje
        &subscription_callback,  // Función callback
        ON_NEW_DATA));       // Solo llamar cuando hay datos nuevos
```

**¿Qué hace?**
- **Executor**: Gestiona todos los callbacks del nodo
- Añade el suscriptor al executor
- Asocia el callback `subscription_callback`
- `ON_NEW_DATA`: Solo ejecuta callback cuando llega mensaje nuevo

```c
    // 7. RESERVAR MEMORIA PARA MENSAJES
    incoming_msg.data.data = (char *)malloc(128 * sizeof(char));
    incoming_msg.data.size = 0;
    incoming_msg.data.capacity = 128;
```

**¿Qué hace?**
- Reserva buffer de 128 bytes para recibir mensajes
- Inicializa estructura String con capacidad

```c
    // 8. LOOP PRINCIPAL
    while (1) {
        rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));  // Procesa callbacks
        vTaskDelay(pdMS_TO_TICKS(100));                         // Espera 100ms
    }
}
```

**¿Qué hace?**
- **Loop infinito** de la tarea FreeRTOS
- `rclc_executor_spin_some()`: Procesa mensajes pendientes
  - Revisa si hay mensajes en la cola
  - Si hay, llama al callback correspondiente
  - Timeout de 100ms
- `vTaskDelay()`: Duerme 100ms para no saturar CPU

---

## 🐍 Análisis del Código Python

### 1. Inicialización del Nodo

```python
class KeyboardLEDController(Node):
    def __init__(self):
        super().__init__('keyboard_led_controller')  # Nombre del nodo
```

**¿Qué hace?**
- Hereda de `Node` (clase base de ROS 2)
- Crea un nodo con nombre `keyboard_led_controller`
- Se registra automáticamente en la red ROS 2

```python
        self.publisher = self.create_publisher(
            String,              # Tipo de mensaje
            '/led_control',      # Nombre del tópico
            10)                  # QoS (tamaño de cola)
```

**¿Qué hace?**
- Crea un publicador para el tópico `/led_control`
- Tipo: `std_msgs/msg/String`
- QoS de 10: Puede almacenar hasta 10 mensajes en cola si no se envían rápido

### 2. Función de Publicación

```python
    def send_command(self, command: str):
        msg = String()          # Crea mensaje vacío
        msg.data = command      # Asigna dato (string)
        self.publisher.publish(msg)  # Publica en tópico
        self.get_logger().info(f'📤 Comando enviado: {command}')
```

**¿Qué hace?**
1. Crea instancia de `String`
2. Asigna el comando al campo `data`
3. Publica el mensaje (envía a todos los suscriptores del tópico)
4. Log en consola

### 3. Lectura de Teclado

```python
    def get_key(self):
        fd = sys.stdin.fileno()
        old_settings = termios.tcgetattr(fd)
        try:
            tty.setraw(sys.stdin.fileno())    # Modo raw (sin buffer)
            ch = sys.stdin.read(1)            # Lee 1 carácter
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        return ch
```

**¿Qué hace?**
- **Modo raw**: Lee tecla inmediatamente sin esperar Enter
- `termios`: Control de terminal Unix
- `tty.setraw()`: Desactiva buffering de línea
- Lee exactamente 1 carácter
- Restaura configuración original

### 4. Loop Principal

```python
    def run(self):
        while True:
            key = self.get_key().lower()
            
            if key == 'e':
                self.send_command('ON')
            elif key == 'a':
                self.send_command('OFF')
            elif key == 't':
                self.send_command('TOGGLE')
            elif key == 'q':
                break
```

**¿Qué hace?**
- Loop infinito
- Lee tecla
- Convierte a minúscula
- Ejecuta acción según tecla
- `break` sale del loop si presionas 'q'

### 5. Main Function

```python
def main():
    rclpy.init()                              # Inicializa ROS 2
    controller = KeyboardLEDController()      # Crea nodo
    
    try:
        controller.run()                      # Ejecuta loop
    except KeyboardInterrupt:
        pass
    finally:
        controller.destroy_node()             # Limpia nodo
        rclpy.shutdown()                      # Cierra ROS 2
```

**¿Qué hace?**
1. `rclpy.init()`: Inicializa el sistema ROS 2
2. Crea instancia del nodo
3. Ejecuta el loop principal
4. Captura Ctrl+C para salir limpiamente
5. Destruye nodo y cierra ROS 2

---

## 🌐 Protocolo de Comunicación

### DDS vs XRCE-DDS

#### DDS (Data Distribution Service)
```
Computadora (ROS 2):
┌──────────────────────────────┐
│ Header: 150 bytes            │
│  - Discovery info            │
│  - QoS policies              │
│  - Timestamp                 │
│  - Source info               │
├──────────────────────────────┤
│ Payload: "ON" (2 bytes)      │
├──────────────────────────────┤
│ Footer: 50 bytes             │
│  - Checksum                  │
│  - Sequence number           │
└──────────────────────────────┘
Total: ~200 bytes
```

#### XRCE-DDS (eXtremely Resource Constrained)
```
ESP32 (micro-ROS):
┌──────────────────────────────┐
│ Header: 12 bytes             │
│  - Session ID                │
│  - Stream ID                 │
│  - Sequence                  │
├──────────────────────────────┤
│ Payload: "ON" (2 bytes)      │
├──────────────────────────────┤
│ Footer: 2 bytes              │
│  - CRC                       │
└──────────────────────────────┘
Total: ~16 bytes
```

**Ahorro: 92%** 🎉

### Establecimiento de Sesión

#### Primer Contacto ESP32 → Agente:

```
1. ESP32 → Agente: CREATE_CLIENT
   ┌────────────────────┐
   │ Type: CREATE       │
   │ Object: CLIENT     │
   │ Client Key: random │
   └────────────────────┘

2. Agente → ESP32: STATUS_OK
   ┌────────────────────┐
   │ Status: OK         │
   │ Session ID: 0x81   │
   │ Client Key: 0x2C.. │
   └────────────────────┘

3. ESP32 → Agente: CREATE_PARTICIPANT
   (crea participante DDS)

4. ESP32 → Agente: CREATE_SUBSCRIBER
   (crea suscriptor)

5. ESP32 → Agente: REQUEST_DATA
   (pide datos del tópico)

6. Agente → ESP32: DATA (cuando llega mensaje)
```

### Flujo de Mensaje Completo

```
Python Script              Agente              ESP32
     │                        │                  │
     │  String("ON")          │                  │
     ├───────DDS──────────────>│                  │
     │                        │                  │
     │                        │  XRCE-DDS        │
     │                        ├─────UDP─────────>│
     │                        │                  │
     │                        │                  │ Callback()
     │                        │                  │ led_on()
     │                        │                  │ ✅
     │                        │                  │
     │                        │  ACK             │
     │                        │<──────UDP────────┤
     │                        │                  │
```

---

## 🎓 Conceptos Clave

### 1. Nodo (Node)
**Definición:** Proceso independiente en la red ROS.

**Características:**
- Tiene un nombre único
- Puede publicar y/o suscribirse a tópicos
- Puede ofrecer/consumir servicios
- Se ejecuta en su propio proceso/thread

**En este proyecto:**
- `keyboard_led_controller` (Python)
- `led_controller` (ESP32)

### 2. Tópico (Topic)
**Definición:** Canal de comunicación con nombre.

**Características:**
- Comunicación asíncrona
- Muchos-a-muchos (N publicadores, M suscriptores)
- Desacoplamiento (no necesitan conocerse)

**En este proyecto:**
- `/led_control` (tipo: std_msgs/msg/String)

### 3. Mensaje (Message)
**Definición:** Estructura de datos que se envía por tópicos.

**Características:**
- Definido en archivos `.msg`
- Serializable
- Con tipos primitivos y compuestos

**Ejemplo: String.msg**
```
string data
```

### 4. Publicador (Publisher)
**Definición:** Envía mensajes a un tópico.

**Características:**
- Múltiples publicadores por tópico permitidos
- No sabe quién recibe

**En este proyecto:**
- Python script publica en `/led_control`

### 5. Suscriptor (Subscriber)
**Definición:** Recibe mensajes de un tópico.

**Características:**
- Múltiples suscriptores por tópico permitidos
- Callback cuando llega mensaje
- No sabe quién envía

**En este proyecto:**
- ESP32 suscrito a `/led_control`

### 6. Callback
**Definición:** Función que se ejecuta cuando ocurre un evento.

**Características:**
- Asíncrono
- Ejecutado por el executor
- Debe ser rápido (no bloquear)

**En este proyecto:**
- `subscription_callback()` en ESP32

### 7. Executor
**Definición:** Gestiona la ejecución de callbacks.

**Características:**
- Procesa cola de mensajes
- Llama callbacks apropiados
- Puede ser single-thread o multi-thread

**En este proyecto:**
- `rclc_executor` en ESP32
- `rclpy.spin()` sería el equivalente en Python (no usado aquí)

### 8. QoS (Quality of Service)
**Definición:** Políticas de confiabilidad de la comunicación.

**Tipos:**
- **Reliable**: Garantiza entrega (TCP-like)
- **Best Effort**: Intenta enviar (UDP-like)

**Otros parámetros:**
- **History**: Cuántos mensajes mantener en cola
- **Durability**: ¿Guardar para nuevos suscriptores?
- **Lifespan**: ¿Cuánto tiempo es válido el mensaje?

**En este proyecto:**
- QoS por defecto (Best Effort, History 10)

---

## 🔬 Debugging y Monitoreo

### Ver Todo el Sistema en Acción

```bash
# Terminal 1: Agente con verbose
ros2 run micro_ros_agent micro_ros_agent udp4 --port 8888 -v

# Terminal 2: Monitor del ESP32
idf.py -p /dev/ttyUSB1 monitor

# Terminal 3: Ver nodos
watch -n 1 ros2 node list

# Terminal 4: Ver mensajes del tópico
ros2 topic echo /led_control

# Terminal 5: Script de control
python3 keyboard_led_control.py
```

### Herramientas de Diagnóstico

```bash
# Info del nodo
ros2 node info /led_controller

# Info del tópico
ros2 topic info /led_control

# Frecuencia de publicación
ros2 topic hz /led_control

# Estructura del mensaje
ros2 interface show std_msgs/msg/String

# Gráfico del sistema
rqt_graph
```

---

## 📊 Resumen Visual

```
╔════════════════════════════════════════════════════════════╗
║                    SISTEMA COMPLETO                        ║
╠════════════════════════════════════════════════════════════╣
║                                                            ║
║  👤 Usuario presiona "E"                                   ║
║           ↓                                                ║
║  🐍 Python: KeyboardLEDController.send_command("ON")       ║
║           ↓                                                ║
║  📡 ROS 2 DDS: Serializa y envía mensaje                   ║
║           ↓                                                ║
║  🔄 Agente: Traduce DDS → XRCE-DDS                        ║
║           ↓                                                ║
║  📶 UDP: Envía paquete a 192.168.0.69                      ║
║           ↓                                                ║
║  📡 ESP32: XRCE-DDS Client recibe                          ║
║           ↓                                                ║
║  ⚙️  Executor: Procesa mensaje                             ║
║           ↓                                                ║
║  📞 Callback: subscription_callback("ON")                  ║
║           ↓                                                ║
║  🔌 GPIO: gpio_set_level(2, 1)                             ║
║           ↓                                                ║
║  💡 LED: ¡ENCIENDE! ✨                                      ║
║                                                            ║
║  Tiempo total: ~50ms                                       ║
╚════════════════════════════════════════════════════════════╝
```

---

## 🎯 Conclusión

Este sistema demuestra:

1. **Comunicación distribuida**: Diferentes dispositivos colaborando
2. **Abstracción**: Python y C++ se comunican sin problemas
3. **Escalabilidad**: Fácil añadir más nodos
4. **Tiempo real**: Latencia baja (~50ms)
5. **Robustez**: Manejo de desconexiones

**La magia de ROS 2:** No necesitas preocuparte por:
- ❌ Sockets
- ❌ Serialización
- ❌ Descubrimiento de nodos
- ❌ Gestión de conexiones

**Todo está abstraído** para que te concentres en la lógica de tu aplicación. 🚀

---

**Documentado por:** Lab ROS 2 - Biofloc Engineering Team  
**Fecha:** Febrero 4, 2026  
**Versión:** 1.0
