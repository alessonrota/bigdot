# Roçambola — Robô da Roça Quilombola - Big Dot


## Estrutura de pastas

```
Raiz-Root
│  readme.md.txt                 ← este arquivo (pode virar README.md)
│
├─ arduino_uno/                  ← reservado para exemplos/testes com Arduino UNO (vazio)
└─ bigdot/
   └─ Tainã/
      ├─ private_settings.h      ← credenciais (Wi‑Fi/MQTT) – preencha aqui
      └─ teste_1.ino             ← sketch principal (Big Dot v2 / SAMD21)
```

## Instalação da placa **Big Dot** com Arduino IDE
Sistema de monitoramento + irrigação agendada usando **Big Dot v2 (SAMD21)**, PMS7003, BME280, OLED SH1106 e Wi‑Fi (ESP‑12F com AT).  
A irrigação automática ocorre **às 07:00 e 19:00 por 20 minutos**. O **botão manual** liga/desliga a válvula, com **tempo máximo de 20 minutos** por acionamento.

Para instalar os arquivos de suporte da **librelab::Big Dot** no Arduino, adicione a seguinte linha no campo **“URLs Adicionais para o Gerenciador de Placas”** (menu **Preferências**) da sua Arduino IDE:

```
https://unixjazz.org/pub/software/package_librelab_index.json
```

Em seguida: **Ferramentas → Placa → Gerenciador de Placas** → procure por **Big Dot** e instale o pacote. Depois selecione a placa adequada (ex.: *Big Dot v2* ou equivalente).

## Hardware (visão geral)

- **Placa**: Big Dot v2 (SAMD21).  
- **Wi‑Fi**: ESP‑12F (AT 1.7.4.0) em **SERCOM2** (Serial2).
- **Display**: OLED 1.3" SH1106 via **I²C**.  
- **Atmosfera**: BME280 (I²C, addr **0x77**).  
- **PM**: PMS7003 em **Serial1**; pino **RST** ligado ao **D2**.  
- **Relé/Válvula**: controle no **D1**.  
- **Botão manual**: **D3**, modo **INPUT_PULLUP** (aciona ao ligar ao GND).  
- **RTC**: PCF8523 via I²C (referência de tempo para a agenda).

> **Energia**: use fonte estável de 5 V com folga de corrente (ESP + PMS + relé têm picos). Compartilhe **GND** entre todos os módulos.

## Funcionalidades

- **Irrigação automática**: liga 07:00 e 19:00 (janela de ~2 min) e desliga ao completar **20 min**.  
- **Irrigação manual** (botão em D3): pressiona → **liga** (até 20 min); pressiona de novo → **desliga**.  
- **Leituras** (a cada ~30 s):  
  - BME280: umidade (%), temperatura (°C), pressão (hPa).  
  - PMS7003: PM2.5 e PM10 (µg/m³).  
  - Exibição no OLED + **publicação MQTT**.
- **Solo desativado**: leitura de umidade do solo está desligada (lógica por agenda/botão).

## Tópicos MQTT (padrão)

- `"/tc/sensor_2/umidade"`  
- `"/tc/sensor_2/temperatura"`  
- `"/tc/sensor_2/pressao"`  
- `"/tc/sensor_1/pm2_5"`  
- `"/tc/sensor_1/pm10"`

## Credenciais (`private_settings.h`)

Edite/crie `bigdot\Tainã\private_settings.h` com:

```cpp
#pragma once
#define WLAN_SSID       "SEU_SSID"
#define WLAN_PASS       "SUA_SENHA"

#define MQTT_SERVER     "broker.exemplo.com"
#define MQTT_PORT       1883
#define MQTT_CLIENT_ID  "taina-bigdot-01"
#define MQTT_USERNAME   "usuario"
#define MQTT_PASSWD     "senha"
```

> **Não** faça commit desse arquivo em repositórios públicos (contém segredos).

## Compilação & Upload

1. **Arduino IDE** (>= 2.x)  
   - Instale o pacote da **Big Dot** (ver seção acima) *ou* “Arduino SAMD Boards (32‑bits ARM Cortex‑M0+)” se estiver usando uma variant equivalente.  
   - **Placa**: selecione *Big Dot v2* (ou *Arduino Zero (Native USB Port)*, conforme seu setup).
2. **Bibliotecas** (versões de referência):  
   - PMserial **1.2.0**  
   - U8g2 **2.31.2**  
   - SparkFun BME280 **2.0.9**  
   - WiFiEspAT **1.5.0**  
   - PubSubClient **2.8.0**  
   - RTClib **2.1.4**
3. Abra `bigdot\Tainã\teste_1.ino` e **Compile/Upload**.

> Se aparecer o aviso “PMserial incompatível com samd”, ajuste `architectures=*` em `library.properties` da PMserial. Geralmente é apenas aviso e compila normalmente.

## Pinos relevantes (padrão do sketch)

| Nome          | Pino | Observação                               |
|---------------|------|------------------------------------------|
| Relé          | D1   | Saída para o módulo de relé              |
| PMS7003 RST   | D2   | Reset do PMS                             |
| Botão manual  | D3   | **INPUT_PULLUP** (GND = pressionado)     |
| ESP‑12F RX    | D6   | `PIN_SERIAL2_RX = 15`                    |
| ESP‑12F TX    | D5   | `PIN_SERIAL2_TX = 14`                    |
| I²C (OLED/RTC/BME)| SDA/SCL | Use os pinos I²C nativos da placa |

Se alterar RX/TX do ESP‑12F, ajuste `PIN_SERIAL2_RX/TX` e as chamadas `pinPeripheral(...)` no código.

## Ajustes rápidos (código)

- **Horários**: edite `handleSchedule()` e troque `7`/`19`.  
- **Duração**: mude `maxRelayTime` (segundos), padrão `20*60`.  
- **Botão**: confira se está no **D3**; se não, ajuste `BUTTON_PIN`.

## Uso com fonte externa (sem USB)

O sketch não bloqueia a ausência de USB (sem `while (!Serial)`). Ao alimentar por fonte externa:
- Garanta **5 V estáveis** e corrente suficiente.  
- **GND comum** entre módulos.  
- Para relé/solenóide, prefira módulo com transistor e diodo de flyback, ou fonte separada da carga.

## Solução de problemas

- **Wi‑Fi/MQTT**: verifique `private_settings.h`, SSID/senha, host/porta e credenciais do broker.  
- **Hora errada**: o RTC ajusta na primeira inicialização com a hora de compilação; ajuste manual se necessário.  
- **Display sem texto**: confira I²C (SDA/SCL), alimentação e endereço do BME280 (0x77).  
- **PMS sem leituras**: aguarde ~3 s de estabilização; revise TX/RX (Serial1) e o pino RST (D2).

## Licença / Créditos

Projeto NPDD da **Casa de Cultura Tainã**. Bibliotecas e marcas pertencem aos respectivos autores.
