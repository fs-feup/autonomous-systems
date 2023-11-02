# Introdução

## Este documento pretende:

- **listar as funções que temos de utilizar e estruturas de dados mais importantes;** [Funções e estruturas de dados](#funções-e-estruturas-de-dados)
- **listar os comandos para correr o tester;** [Tester](#para-correr-o-tester)
- **listar os sinais do CAN_B bus usados para as comunicações entre o AI Computer e o Vehicle Control Unit no ADS_DV;** [CAN messages](#mensagens-can)
- [Link para o repositório no GitHub](https://github.com/FS-AI/FS-AI_API/tree/main) **para a informação original**

## Funções e estruturas de dados

**O envio e receção dos dados é feito através do CAN bus e por funções já implementadas que só temos de chamar com as estruturas de dados certas, ao contrário do que era feito antes já não nos é fornecida a library(.a) mas sim todo o source code, o que significa que temos a liberdade de fazer alterações se necesário:**
### fs-ai_api_init

```c
int fs-ai_api_init(char *CAN_interface, int debug, int simulate);
```

Função que inicia a biblioteca.

#### Parâmetros

- `char *CAN_interface`: String que referencia uma interface CAN válida.

- `int debug`:
  - Se `debug` for diferente de 0: O output do debugging fica no stdout.
  - De outra forma: A biblioteca não é utilizada.

- `int simulate`:
  - Se `simulate` for diferente de 0: Os valores de simulação são carregados para a estrutura de dados.
  - De outra forma: Nada acontece.

#### Retorno

- `int`: 
  - `EXIT_SUCCESS` (0) em caso de sucesso.
  - `EXIT_FAILURE` (8) em caso de falha.

**Nota:** Estes dados não representam a operação do carro e mudam a cada chamada de `fs_ai_api_vcu2ai_get_data()`.

### fs_ai_api_vcu2ai_get_data

```c
void fs_ai_api_vcu2ai_get_data(fs_ai_api_vcu2ai *data);
```

Povoa a estrutura de dados `fs_ai_api_vcu2ai` com os últimos dados recebidos do veículo. A recepção dos dados é feita de forma assíncrona e armazenada temporariamente conforme vai sendo recebido.

#### Parâmetros

- `fs_ai_api_vcu2ai *data`: Estrutura de dados que será populada.
```c
typedef volatile struct fs_ai_api_vcu2ai_struct {
	volatile _Alignas(4) fs_ai_api_handshake_receive_bit_e	VCU2AI_HANDSHAKE_RECEIVE_BIT; // _Alignas(x) usado por cause de requerimentos de memória da VCU, não é muito relevante
	volatile _Alignas(4) fs_ai_api_res_go_signal_bit_e		VCU2AI_RES_GO_SIGNAL;
	volatile _Alignas(4) fs_ai_api_as_state_e				VCU2AI_AS_STATE;
	volatile _Alignas(4) fs_ai_api_ami_state_e				VCU2AI_AMI_STATE;
	volatile _Alignas(4) float								VCU2AI_STEER_ANGLE_deg;
	volatile _Alignas(4) float								VCU2AI_BRAKE_PRESS_F_pct;
	volatile _Alignas(4) float								VCU2AI_BRAKE_PRESS_R_pct;
	volatile _Alignas(4) float								VCU2AI_FL_WHEEL_SPEED_rpm;
	volatile _Alignas(4) float								VCU2AI_FR_WHEEL_SPEED_rpm;
	volatile _Alignas(4) float								VCU2AI_RL_WHEEL_SPEED_rpm;
	volatile _Alignas(4) float								VCU2AI_RR_WHEEL_SPEED_rpm;
	volatile _Alignas(4) uint16_t							VCU2AI_FL_PULSE_COUNT;
	volatile _Alignas(4) uint16_t							VCU2AI_FR_PULSE_COUNT;
	volatile _Alignas(4) uint16_t							VCU2AI_RL_PULSE_COUNT;
	volatile _Alignas(4) uint16_t							VCU2AI_RR_PULSE_COUNT;
} fs_ai_api_vcu2ai;
```
**Esta estrutura esta dependente das seguintes ( o nome dos vários elementos é por vezes suficiente para perceber o seu papel, porém, nem sempre, como tal mais a baixo no documento temos uma descrição mais pormenorizada de todos os sinais)**
```c
typedef enum fs_ai_api_ami_state_e {
	AMI_NOT_SELECTED = 0,
	AMI_ACCELERATION = 1,
	AMI_SKIDPAD = 2,
	AMI_AUTOCROSS = 3,
	AMI_TRACK_DRIVE = 4,
	AMI_STATIC_INSPECTION_A = 5,
	AMI_STATIC_INSPECTION_B = 6,
	AMI_AUTONOMOUS_DEMO = 7,
} fs_ai_api_ami_state_e;

typedef enum fs_ai_api_as_state_e {
	AS_OFF = 1,
	AS_READY = 2,
	AS_DRIVING = 3,
	AS_EMERGENCY_BRAKE = 4,
	AS_FINISHED = 5,
} fs_ai_api_as_state_e;

typedef enum fs_ai_api_res_go_signal_bit_e {
	RES_GO_SIGNAL_NO_GO = 0,
	RES_GO_SIGNAL_GO = 1,
} fs_ai_api_res_go_signal_bit_e;

typedef enum fs_ai_api_handshake_receive_bit_e {
	HANDSHAKE_RECEIVE_BIT_OFF = 0,
	HANDSHAKE_RECEIVE_BIT_ON = 1,
} fs_ai_api_handshake_receive_bit_e;

```
### fs_ai_api_ai2vcu_set_data

```c
void fs_ai_api_ai2vcu_set_data(fs_ai_api_ai2vcu *data);
```
A periodicidade ideal de chamamento da função é 10ms, sendo que existe um timer interno que evita que esse período desça para menos de 8ms (evitar overload do CAN bus). Além disso, períodos muito grandes ativam o  CAN timeout diagnostics da ECU (Eletronics Control Unit).

Transmite os frames de CAN para o VCU.

#### Parâmetros

`fs_ai_api_ai2vcu *data`: Estrutura de dados com os dados que serão passados para o VCU.

```c
#ifdef __cplusplus // caso C++
typedef volatile struct alignas(4) fs_ai_api_ai2vcu_struct {
	volatile fs_ai_api_mission_status_e		AI2VCU_MISSION_STATUS;
	volatile fs_ai_api_direction_request_e	AI2VCU_DIRECTION_REQUEST;
	volatile fs_ai_api_estop_request_e		AI2VCU_ESTOP_REQUEST;
	volatile fs_ai_api_handshake_send_bit_e	AI2VCU_HANDSHAKE_SEND_BIT;
	volatile float							AI2VCU_STEER_ANGLE_REQUEST_deg;
	volatile float							AI2VCU_AXLE_SPEED_REQUEST_rpm;
	volatile float							AI2VCU_AXLE_TORQUE_REQUEST_Nm;
	volatile float							AI2VCU_BRAKE_PRESS_REQUEST_pct;
} fs_ai_api_ai2vcu;
#else // C
typedef volatile struct fs_ai_api_ai2vcu_struct {
	volatile _Alignas(4) fs_ai_api_mission_status_e		AI2VCU_MISSION_STATUS;
	volatile _Alignas(4) fs_ai_api_direction_request_e	AI2VCU_DIRECTION_REQUEST;
	volatile _Alignas(4) fs_ai_api_estop_request_e		AI2VCU_ESTOP_REQUEST;
	volatile _Alignas(4) fs_ai_api_handshake_send_bit_e	AI2VCU_HANDSHAKE_SEND_BIT;
	volatile _Alignas(4) float							AI2VCU_STEER_ANGLE_REQUEST_deg;
	volatile _Alignas(4) float							AI2VCU_AXLE_SPEED_REQUEST_rpm;
	volatile _Alignas(4) float							AI2VCU_AXLE_TORQUE_REQUEST_Nm;
	volatile _Alignas(4) float							AI2VCU_BRAKE_PRESS_REQUEST_pct;
} fs_ai_api_ai2vcu;
#endif
```
### fs_ai_api_imu_get_data

```c
void fs_ai_api_imu_get_data(fs_ai_api_imu *data);
```
Povoa a estrutura de dados `fs_ai_api_imu` com os últimos dados recebidos do PCAN-GPS que está equipado no veículo. A recepção dos dados é feita de forma assíncrona e armazenada temporariamente conforme vai sendo recebido.
[Informações extra sobre os argumentos na página 36 (e seguintes) deste documento](https://github.com/FS-AI/FS-AI_API/blob/main/Docs/PCAN-GPS_UserMan_eng.pdf)

#### Parâmetros
`fs_ai_api_imu *data`
```c
#ifdef __cplusplus
typedef volatile struct alignas(4) fs_ai_api_imu_struct {
	volatile float		IMU_Acceleration_X_mG;
	volatile float		IMU_Acceleration_Y_mG;
	volatile float		IMU_Acceleration_Z_mG;
	volatile float		IMU_Temperature_degC;
	volatile uint8_t	IMU_VerticalAxis;
	volatile uint8_t	IMU_Orientation;
	volatile float		IMU_MagneticField_X_uT;
	volatile float		IMU_MagneticField_Y_uT;
	volatile float		IMU_MagneticField_Z_uT;
	volatile float		IMU_Rotation_X_degps;
	volatile float		IMU_Rotation_Y_degps;
	volatile float		IMU_Rotation_Z_degps;
} fs_ai_api_imu;
#else
typedef volatile struct fs_ai_api_imu_struct {
	volatile _Alignas(4) float		IMU_Acceleration_X_mG;
	volatile _Alignas(4) float		IMU_Acceleration_Y_mG;
	volatile _Alignas(4) float		IMU_Acceleration_Z_mG;
	volatile _Alignas(4) float		IMU_Temperature_degC;
	volatile _Alignas(4) uint8_t	IMU_VerticalAxis;
	volatile _Alignas(4) uint8_t	IMU_Orientation;
	volatile _Alignas(4) float		IMU_MagneticField_X_uT;
	volatile _Alignas(4) float		IMU_MagneticField_Y_uT;
	volatile _Alignas(4) float		IMU_MagneticField_Z_uT;
	volatile _Alignas(4) float		IMU_Rotation_X_degps;
	volatile _Alignas(4) float		IMU_Rotation_Y_degps;
	volatile _Alignas(4) float		IMU_Rotation_Z_degps;
} fs_ai_api_imu;
#endif
```

### fs_ai_api_gps_get_data

```c
void fs_ai_api_gps_get_data(fs_ai_api_gps *data);
```

Povoa a estrutura de dados `fs_ai_api_gps` com os últimos dados recebidos do PCAN-GPS que está equipado no veículo. A recepção dos dados é feita de forma assíncrona e armazenada temporariamente conforme vai sendo recebido. A única diferença em relação à função anterior é o fato de ser uma estrutura de dados diferente.
[Informações extra sobre os argumentos na página 36 (e seguintes) deste documento](https://github.com/FS-AI/FS-AI_API/blob/main/Docs/PCAN-GPS_UserMan_eng.pdf)

```c
#ifdef __cplusplus
typedef volatile struct alignas(4) fs_ai_api_gps_struct {
	volatile uint8_t	GPS_AntennaStatus;
	volatile uint8_t	GPS_NumSatellites;
	volatile uint8_t	GPS_NavigationMethod;
	volatile float		GPS_Course_deg;
	volatile float		GPS_Speed_kmh;
	volatile float		GPS_Longitude_Minutes;
	volatile uint16_t	GPS_Longitude_Degree;
	volatile uint8_t	GPS_Longitude_IndicatorEW;
	volatile float		GPS_Latitude_Minutes;
	volatile uint16_t	GPS_Latitude_Degree;
	volatile uint8_t	GPS_Latitude_IndicatorNS;
	volatile float		GPS_Altitude;
	volatile float		GPS_PDOP;
	volatile float		GPS_HDOP;
	volatile float		GPS_VDOP;
	volatile uint8_t	GPS_UTC_Year;
	volatile uint8_t	GPS_UTC_Month;
	volatile uint8_t	GPS_UTC_DayOfMonth;
	volatile uint8_t	GPS_UTC_Hour;
	volatile uint8_t	GPS_UTC_Minute;
	volatile uint8_t	GPS_UTC_Second;
} fs_ai_api_gps;
#else
typedef volatile struct fs_ai_api_gps_struct {
	volatile _Alignas(4) uint8_t	GPS_AntennaStatus;
	volatile _Alignas(4) uint8_t	GPS_NumSatellites;
	volatile _Alignas(4) uint8_t	GPS_NavigationMethod;
	volatile _Alignas(4) float		GPS_Course_deg;
	volatile _Alignas(4) float		GPS_Speed_kmh;
	volatile _Alignas(4) float		GPS_Longitude_Minutes;
	volatile _Alignas(4) uint16_t	GPS_Longitude_Degree;
	volatile _Alignas(4) uint8_t	GPS_Longitude_IndicatorEW;
	volatile _Alignas(4) float		GPS_Latitude_Minutes;
	volatile _Alignas(4) uint16_t	GPS_Latitude_Degree;
	volatile _Alignas(4) uint8_t	GPS_Latitude_IndicatorNS;
	volatile _Alignas(4) float		GPS_Altitude;
	volatile _Alignas(4) float		GPS_PDOP;
	volatile _Alignas(4) float		GPS_HDOP;
	volatile _Alignas(4) float		GPS_VDOP;
	volatile _Alignas(4) uint8_t	GPS_UTC_Year;
	volatile _Alignas(4) uint8_t	GPS_UTC_Month;
	volatile _Alignas(4) uint8_t	GPS_UTC_DayOfMonth;
	volatile _Alignas(4) uint8_t	GPS_UTC_Hour;
	volatile _Alignas(4) uint8_t	GPS_UTC_Minute;
	volatile _Alignas(4) uint8_t	GPS_UTC_Second;
} fs_ai_api_gps;
#endif
```
### fs_ai_api_get_can_stats

```c
void fs_ai_api_get_can_stats(can_stats_t *data);
```

Povoa a estrutura de dados `can_stats_t` com os últimos valores recebidos do veículo. Os dados não são recebidos de forma assíncrona e é utilizado para debugging.
```c
typedef struct can_stats_struct {
	volatile uint32_t VCU2AI_Status_count;
	volatile uint32_t VCU2AI_Drive_F_count;
	volatile uint32_t VCU2AI_Drive_R_count;
	volatile uint32_t VCU2AI_Steer_count;
	volatile uint32_t VCU2AI_Brake_count;
	volatile uint32_t VCU2AI_Wheel_speeds_count;
	volatile uint32_t VCU2AI_Wheel_counts_count;
	volatile uint32_t PCAN_GPS_BMC_Acceleration_count;
	volatile uint32_t PCAN_GPS_BMC_MagneticField_count;
	volatile uint32_t PCAN_GPS_L3GD20_Rotation_A_count;
	volatile uint32_t PCAN_GPS_L3GD20_Rotation_B_count;
	volatile uint32_t PCAN_GPS_GPS_Status_count;
	volatile uint32_t PCAN_GPS_GPS_CourseSpeed_count;
	volatile uint32_t PCAN_GPS_GPS_Longitude_count;
	volatile uint32_t PCAN_GPS_GPS_Latitude_count;
	volatile uint32_t PCAN_GPS_GPS_Altitude_count;
	volatile uint32_t PCAN_GPS_GPS_Delusions_A_count;
	volatile uint32_t PCAN_GPS_GPS_Delusions_B_count;
	volatile uint32_t PCAN_GPS_GPS_DateTime_count;
	volatile uint32_t unhandled_frame_count;
} can_stats_t;
```

### fs_ai_api_clear_can_stats

```c
void fs_ai_api_clear_can_stats();
```

Limpa a `can_stats_t` (estrutura de dados anterior) de quaisquer dados.





## Para correr o tester

Utilizando uma CAN virtual (vcan0, por exemplo) podemos correr o tester fornecido que dá display ás informações da VCU ;

Portanto para testar sem hardware:

**build and run:**

**1-**	
	
    git clone git@github.com:FS-AI/FS-AI_API.git
    

 
   clone do repositório


**2-**

	cd FS-AI_API/
	
verificar que estamos no diretório principal, que tem: 
	
	\Docs
		(documentation files)
	\FS-AI_API
		(library source code & makefile)
	\FS-AI_API_Console
		(Console test programme)
	\FS-AI_API_Tester
		(Test programme)
	\images
		(images for Markdown files)

**3-**  

	cd FS-AI_API_Tester  
	
   or
	
	cd FS-AI_API_Console


**4-**	

	make  
   to build (library will build / rebuild as needed).

**5-**


    ./fs-ai_api_tester vcan0      
    
   
   to run on vcan0.


**Em caso de aparecer este erro**:
Called fs_ai_api_init(vcan0, 1, 1)

Simulate Mode enabled...

Error in ioctl(): No such device

Can't open [vcan0]fs_ai_api_init() failed


**6-** 

    sudo ip link add dev vcan0 type vcan
    sudo ip link set vcan0 up


**7-** 

	./fs-ai_api_tester vcan0 

to run on vcan0.


To bring down the link to reset:
- `sudo ip link set vcan0 down`
- `sudo ip link delete vcan0`

To run again:
- `sudo ip link add dev vcan0 type vcan`
- `sudo ip link set vcan0 up`
- `step 5`


Caso tenha havido erros tentar correr:

- sudo modprobe can_dev
- sudo modprobe can
- sudo modprobe can_raw
- sudo modprobe vcan
- sudo apt-get install can-utils
- e voltar ao step 2

## Mensagens CAN

### Mensagens do AI Computer

**AI2LOG_Dynamics2:** Mensagem dinâmica do veículo para cumprir os requisitos do registo dos dados.

| Sinal     | Tipo     | Descrição                  |
| :-------- | :------- | :------------------------- |
| `Accel_longitudinal_ms2` | `Signed` | Aceleração longitudinal (m/s2) |
| `Accel_lateral_ms2` | `Signed` | Aceleração lateral do veículo (m/s2) |
| `Yaw_rate_degpd` | `Signed` | Taxa de yaw (º/s) |

**AI2VCU_Status:** Sinais de estado do AI Computer para o VCU e alguns sinais de registo FSG

| Sinal.    | Tipo     | Descrição                         |
| :-------- | :------- | :-------------------------------- |
| `HANDSHAKE`      | `Unsigned` |Bit do handshake para verificar as comunicações entre o AI Computer e o VCU|
| `ESTOP_REQUEST`      | `Enum` |Pedido do AI para o emergency stop do veículo|
|||0 = E-stop não pedido|
|||1 = E-stop pedido|
|`MISSION_STATUS`|`Enum` | Estado do autonomous mission para o VCU determinar que estado do autonomous driving deve estar |
|||0 = not selected|
|||1 = mission selected|
|||2 = running|
|||3 = finished|
| `DIRECTION_REQUEST`      | `Enum` |Pedido de direção do veículo|
|||0 = neutro|
|||1 = avançar|
| `LAP_COUNTER`      | `Unsigned` |Contador de voltas para o "Track drive" mission|
|`CONES_COUNT_ACTUAL`|`Unsigned`|Número de cones que foram detetados momentaneamente|
|`CONES_COUNT_ALL`|`Unsigned`|Contador de cones detetados|
|`VEH_SPEED_ACTUAL`|`Unsigned`|Velocidade atual do veículo|
|`VEH_SPEED_DEMAND`|`Unsigned`| Velocidade exigida ao veículo|

**AI2VCU_Drive_F:** Pedidos para o motor da frente

| Sinal.    | Tipo     | Descrição                         |
| :-------- | :------- | :-------------------------------- |
| `FRONT_AXLE_TRQ_REQUEST`      | `Unsigned` |O torque absoluto do eixo dianteiro que foi solicidado. O torque é positivo ou negativo dependento do limite de velocidade do motor|
|||Intervalo: [0, 195]|
|`FRONT_MOTOR_SPEED_MAX`|`Unsigned`|Velocidade máxima do motor para o controlo de velocidade do veículo|
|||Intervalo: [0, 4000]|

**AI2VCU_Drive_R:** Pedidos para o motor de tração traseira

| Sinal.    | Tipo     | Descrição                         |
| :-------- | :------- | :-------------------------------- |
| `FRONT_AXLE_TRQ_REQUEST`      | `Unsigned` |O torque absoluto do eixo traseiro que foi solicidado|
|||Intervalo: [0, 195]|
|`FRONT_MOTOR_SPEED_MAX`|`Unsigned`|Velocidade máxima do motor para o controlo de velocidade do veículo|
|||Intervalo: [0, 4000]|

**AI2VCU_Steer:** Solicitação do ângulo de direção

| Sinal.    | Tipo     | Descrição                         |
| :-------- | :------- | :-------------------------------- |
| `FRONT_AXLE_TRQ_REQUEST`      | `Unsigned` |Solicitação do ângulo de direção. Um ângulo positivo move as rodas da frente para a esquerda de acordo com o ISO 8855 sistema de coordenadas|

![Steering example image place holder](../assets/API/steering.png)


**AI2VCU_Brake:** Pedido de pressão do travão hidráulico 

| Sinal.    | Tipo     | Descrição                         |
| :-------- | :------- | :-------------------------------- |
| `HYD_PRESS_F_REQ_pct`      | `Unsigned` |Pedido normalizado de pressão hidráulica para os travões de fricção do eixo dianteiro.|
|||Intervalo: [0, 100]|
| `HYD_PRESS_R_REQ_pct`      | `Unsigned` |Pedido normalizado de pressão hidráulica para os travões de fricção do eixo traseiro.|
|||Intervalo: [0, 100]|

### Mensagens do AI Vehicle Control Unit (VCU)

**VCU2LOG_Dynamics1:** Mensagem dinâmica do veículo para cumprir os requisitos do registo dos dados.

| Sinal     | Tipo     | Descrição                  |
| :-------- | :------- | :------------------------- |
| `Speed_actual_kmh` | `Unsigned` | Velocidade real do veículo |
| `Speed_target_kmh` | `Unsigned` | Velocidade alvo do veículo |
| `Steer_actual_deg` | `Signed` | Ângulo de direção atual |
| `Steer_target_deg` | `Signed` | Ângulo de direção solicitado |
| `Brake_actual_pct` | `Unsigned` | Percentagem de travagem mecânica |
| `Brake_target_pct` | `Unsigned` | Percentagem de travagem mecânica solicitada |
| `Drive_trq_actual_pct` | `Signed` | Soma do torque real do eixo dianteiro e traseiro. |
| `Drive_trq_target_pct` | `Signed` | Soma do torque pedido do eixo dianteiro e traseiro. |

**VCU2LOG_Status:** Relatório de status do veículo

| Sinal     | Tipo     | Descrição                  |
| :-------- | :------- | :------------------------- |
| `State_ASSI` | `Enum` | Indicador do estado do Autonomous System Status |
|||1 = AS_OFF|
|||2 = AS_READY|
|||3 = AS_DRIVING|
|||4 = EMERGENCY_BRAKE|
|||5 = AS_FINISHED|
| `State_EBS` | `Enum` | Estado do Emergency Braking System |
|||1 = unavailable|
|||2 = armed|
|||3 = triggered|
| `AMI_STATE` | `Enum` | Estado do Autonomous Mission Indicator (AMI) |
|||0 = not selected|
|||1 = Acceleration|
|||2 = Skidpad|
|||3 = Autocross|
|||4 = Track drive|
|||5 = Static Inspection A|
|||6 = Static Inspection B|
|||7 = Autonomous demo|
| `State_steering` | `Enum` | Status do Steering System |
|||0 = off|
|||1 = active|
| `State_service_brake` | `Enum` | Estado dos travões de serviço (travões mecânicos de fricção): |
|||1 = disengaged|
|||2 = engaged|
|||3 = available|
| `Lap_counter` | `Unsigned` | Sinal transmitido do AI2VCU_Status através de gateway |
| `Cones_count_actual` | `Unsigned` | Sinal transmitido do AI2VCU_Status através de gateway |
| `Cones_count_all` | `Unsigned` | Sinal transmitido do AI2VCU_Status através de gateway |

**VCU2AI_Status:** Relatório de status do veículo

| Sinal     | Tipo     | Descrição                  |
| :-------- | :------- | :------------------------- |
| `HANDSHAKE` | `Unsigned` | Bit do handshake para verificar as comunicações entre o AI Computer e o VCU |
| `SHUTDOWN_REQUEST` | `enum` | Pedido para desligar o AI Computer |
|||0 = não desliga|
|||1 = pedido para desligar pedido|
| `AS_SWITCH_STATUS` | `enum` | Estado do interruptor principal do sistema autónomo |
|||0 = off|
|||1 = on|
| `TS_SWITCH_STATUS` | `enum` | Estado do interruptor principal do sistema de tração |
|||0 = off|
|||1 = on|
| `GO_SIGNAL` | `enum` |Sinal "Go" para permitir que o veículo comece a avançar |
|||0 = no go|
|||1 = go|
| `STEERING_STATUS` | `enum` | Estado do sistema de direção |
|||0 = off|
|||1 = active|
| `AS_STATUS` | `enum` | Estado do sistema autónomo |
|||1 = AS_OFF|
|||2 = AS_READY|
|||3 = AS_DRIVING|
|||4 = EMERGENCY_BRAKE|
|||5 = AS_FINISHED|
| `AMI_STATE` | `Enum` | Estado do Autonomous Mission Indicator (AMI) |
|||0 = not selected|
|||1 = Acceleration|
|||2 = Skidpad|
|||3 = Autocross|
|||4 = Track drive|
|||5 = Static Inspection A|
|||6 = Static Inspection B|
| `FAULT_STATUS` | `enum` | Flag para indicar a presença de uma falha crítica |
|||0 = sem falhas|
|||1 = falha detetada|
| `WARNING_STATUS` | `enum` | Flag para indicar a presença de um aviso - Usado para o diagnóstico de falhas antes da corrida |
|||0 = sem avisos|
|||1 = aviso ativo|
| `WARN_BATT_TEMP_HIGH` | `enum` | Indicador de aviso de alta temperatura da bateria de tração |
|||0 = sem avisos|
|||1 = aviso ativo|
| `WARN_BATT_SOC_LOW` | `enum` | Indicador de aviso de baixo SOC (State of Charge) da bateria de tração|
|||0 = sem avisos|
|||1 = aviso ativo|
| `AI_ESTOP_REQUEST` | `enum` |Indicador de solicitação de parada de emergência (E-stop) pelo AI computer |
|||0 = inativo|
|||1 = E-stop pedido|
| `HVIL_OPEN_FAULT` | `enum` | Indicador de falha de circuito aberto no circuito de desligamento |
|||0 = sem falha|
|||1 = falha detetada|
| `HVIL_SHORT_FAULT` | `enum` | Indicador de falha de curto-circuito no circuito de desligamento |
|||0 = sem falha|
|||1 = falha detetada|
| `EBS_FAULT` | `enum` | Indicador de falha no EBS |
|||0 = sem falha|
|||1 = falha detetada|
| `OFFBOARD_CHARGER_FAULT` | `enum` | Indicador de falha no carregador de bateria externo |
|||0 = sem falha|
|||1 = falha detetada|
| `AI_COMMS_LOST` | `enum` | Falha de comunicação no CAN |
|||0 = sem falha|
|||1 = falha detetada|
| `AUTONOMOUS_BRAKING_FAULT` | `enum` | Sinal de solicitação de direção NEUTRA feito enquanto o veículo ainda está em movimento |
|||0 = sem falha|
|||1 = falha detetada|
| `MISSION_STATUS_FAULT` | `enum` | MISSION_STATUS foi colocado em "FINISHED" enquanto o veículo se ecnontra em andamento |
|||0 = sem falha|
|||1 = falha detetada|
| `CHARGE_PROCEDURE_FAULT` | `enum` | Os interruptores mestre AS ou TS estão ligados enquanto a bateria está a ser carregada |
|||0 = sem falha|
|||1 = falha detetada|
| `BMS_FAULT` | `enum` | Indicador de falha BMS |
|||0 = sem falha|
|||1 = falha detetada|
| `BRAKE_PLAUSABILITY_FAULT` | `enum` | Na operação de controlo de torque do motor de acionamento, é aplicada uma demanda de torque de acionamento não nula ao mesmo tempo que uma demanda de pressão de travão não nula. |
|||0 = sem falha|
|||1 = falha detetada|
| `SHUTDOWN_CAUSE` | `enum` | Lista enumerada com a indicação da falha que levou o VCU a desligar:|
|||0 = não desligou|
|||1 = pedido do AI Computer|
|||2 = HVIL open-circuit fault|
|||3 = HVIL short-circuit fault|
|||4 = EBS fault|
|||5 = Offboard battery charger fault|
|||6 = AI communications fault |
|||7 = Autonomous braking fault 8 = Mission status fault |
|||9 = Charging procedure fault 10 = BMS fault |
|||11 = Brake plausibility fault |

**VCU2AI_Drive_F:** Feedback do motor da frente

| Sinal.    | Tipo     | Descrição                         |
| :-------- | :------- | :-------------------------------- |
| `FRONT_AXLE_TRQ`  | `Unsigned` |O torque verdadeiro do eixo dianteiro|
|||Intervalo: [-195, 195]|
|`FRONT_AXLE_TRQ_REQUEST`|`Unsigned`|Torque do eixo dianteiro pedido|
|`FRONT_AXLE_TRQ_MAX`|`Unsigned`|Torque máximo permitido no eixo de acionamento|
|||Intervalo: [0, 195]|

**VCU2AI_Drive_R:** Feedback do motor da frente

| Sinal.    | Tipo     | Descrição                         |
| :-------- | :------- | :-------------------------------- |
| `FRONT_AXLE_TRQ`  | `Unsigned` |O torque verdadeiro do eixo traseiro|
|||Intervalo: [-195, 195]|
|`FRONT_AXLE_TRQ_REQUEST`|`Unsigned`|Torque do eixo traseiro pedido|
|`FRONT_AXLE_TRQ_MAX`|`Unsigned`|Torque máximo permitido no eixo de acionamento|
|||Intervalo: [0, 195]|

**VCU2AI_Steer:** Feedback do controlador de direação

| Sinal.    | Tipo     | Descrição                         |
| :-------- | :------- | :-------------------------------- |
| `ANGLE `  | `signed` |Verdadeiro ângulo de direção|
|||Intervalo: [-21.0, 21.0]|
|`ANGLE_MAX`|`Unsigned`|Ângulo máximo permitido igual para ambos os lados|
|`ANGLE_REQUEST`|`Signed`|Ângulo de direção pedido através de (AI2VCU_Steer,STEER_REQUEST)|

**VCU2AI_Brake:** Feedback do sistema hidraulico de travagem

| Sinal.    | Tipo     | Descrição                         |
| :-------- | :------- | :-------------------------------- |
| `HYD_PRESS_F_pct `  | `Unsigned` |Pressão verdadeira no travão da frente|
|||Intervalo: [0, 100]|
| `HYD_PRESS_F_REQ_pct  `  | `Unsigned` |Pedido de alteração da pressão do travão da frente através de ((AI2VCU_Brake, HYD_PRESS_F_REQ_pct))|
| `HYD_PRESS_R_pct `  | `Unsigned` |Pressão verdadeira no travão de trás|
|||Intervalo: [0, 100]|
| `HYD_PRESS_R_REQ_pct  `  | `Unsigned` |Pedido de alteração da pressão do travão de trás através de ((AI2VCU_Brake, HYD_PRESS_R_REQ_pct))|

| `STATUS_BRK` | `enum` | Estado do sistema de travagem|
|||0 = initialising|
|||1 = ready|
|||2 = shutting down|
|||3 = shutdown complete|
|||4 = fault|

**VCU2AI_Speeds:** Feedback do controlador de direação
| Sinal.    | Tipo     | Descrição                         |
| :-------- | :------- | :-------------------------------- |
| `FL_WHEEL_SPEED `  | `Unsigned` |Velocidade do roda esquerdo da frente|
| `FR_WHEEL_SPEED `  | `Unsigned` |Velocidade do roda direito da frente|
| `RL_WHEEL_SPEED `  | `Unsigned` |Velocidade do roda esquerdo da trás|
| `RR_WHEEL_SPEED `  | `Unsigned` |Velocidade do roda direito de trás|

**VCU2AI_Wheel_counts:** Raw pulse count measurements dos sensores de velocidade das rodas

| Sinal.    | Tipo     | Descrição                         |
| :-------- | :------- | :-------------------------------- |
| `FL_PULSE_COUNT `  | `Unsigned` |Pulse counts da roda esquerda da frente|
|||Intervalo: [0, 65535]|
| `FR_PULSE_COUNT `  | `Unsigned` |Pulse counts da roda direita da frente|
|||Intervalo: [0, 65535]|
| `RL_PULSE_COUNT `  | `Unsigned` |Pulse counts da roda esquerda de trás|
|||Intervalo: [0, 65535]|
| `RR_PULSE_COUNT `  | `Unsigned` |Pulse counts da roda direita da frente|
|||Intervalo: [0, 65535]|



**VCU_STATUS:** VCU status and diagnostic data

| Signal                          | Type     | Description                                                      |
| :------------------------------ | :------- | :--------------------------------------------------------------- |
| `SM_SYS`                        | `Enum`   | Active VCU main state machine state:                              |
|                                 |          | 0 = INITIAL_ACTIONS                                              |
|                                 |          | 1 = POWER_ON_SELF_TEST                                           |
|                                 |          | 2 = AUX                                                          |
|                                 |          | 3 = POWERTRAIN_ENABLE                                            |
|                                 |          | 4 = DRIVE_AUTONOMOUS                                             |
|                                 |          | 5 = DRIVE_MANUAL                                                 |
|                                 |          | 6 = CHARGE                                                       |
|                                 |          | 7 = SHUTDOWN                                                     |
|                                 |          | 8 = SHUTDOWN_OFF                                                 |
|                                 |          | 9 = PUSHBAR_MODE                                                 |
| `SM_AS`                         | `Enum`   | Active autonomous driving state:                                 |
|                                 |          | 1 = AS_OFF                                                       |
|                                 |          | 2 = AS_READY                                                     |
|                                 |          | 3 = AS_DRIVING                                                   |
|                                 |          | 4 = AS_EMERGENCY_BRAKE                                           |
|                                 |          | 5 = AS_FINISHED                                                  |
|                                 |          | 6 = AS_R2D                                                       |
| `R1_AI2VCU_STATUS_TIMEOUT_ERROR` | `Bit`    | Timeout error flag for AI2VCU_Status message                     |
| `R1_AI2VCU_DRIVE_F_TIMEOUT_ERROR` | `Bit`    | Timeout error flag for AI2VCU_Drive_F message                     |
| `R1_AI2VCU_DRIVE_R_TIMEOUT_ERROR` | `Bit`    | Timeout error flag for AI2VCU_Drive_R message                     |
| `R1_AI2VCU_STATUS_HANDSHAKE_ERROR`| `Bit`    | Timeout error flag for the AI handshake                           |
| `R1_AI2VCU_STEER_TIMEOUT_ERROR`   | `Bit`    | Timeout error flag for AI2VCU_Steer message                       |
| `R1_AI2VCU_BRAKE_TIMEOUT_ERROR`   | `Bit`    | Timeout error flag for AI2VCU_Brake message                       |
| `SYS_ACTION_STATE`               | `Enum`   | VCU operating mode:                                              |
|                                 |          | 0 = initialise                                                   |
|                                 |          | 1 = battery charging                                             |
|                                 |          | 2 = autonomous driving                                           |
|                                 |          | 3 = manual driving                                               |
|                                 |          | 4 = Shutdown                                               |
| `WARN_BRAKE_PLAUSIBILITY`        | `Bit`    | Warning flag for the brake plausibility fault                    |
| `WARN_KL15_UNDER_V`              | `Bit`    | Warning flag for low 12V battery voltage                         |
| `WARN_AI_ESTOP_REQ`              | `Bit`    | Warning flag for an AI E-stop request                            |
| `WARN_AI_COMMS_LOST`             | `Bit`    | Warning flag for the AI comms lost fault                         |
| `WARN_AUTO_BRAKING`              | `Bit`    | Warning flag for the autonomous braking fault                    |
| `WARN_MISSION_STATUS`            | `Bit`    | Warning flag for the mission status fault                        |


