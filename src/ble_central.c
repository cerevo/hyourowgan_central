#include <stdio.h>
#include "TZ10xx.h"
#include "PMU_TZ10xx.h"
#include "GPIO_TZ10xx.h"
#include "RNG_TZ10xx.h"

#include "twic_interface.h"
#include "blelib.h"

#include "TZ01_console.h"

static uint8_t msg[80];

static uint64_t hrgn_bdaddr  = 0xc00200000000;   //

static uint8_t bnmsg_gap_device_name[] = "HRGCent00";

static uint8_t bnmsg_gap_appearance[] = {0x00, 0x00};

static const uint8_t bnmsg_di_manufname[] = "Cerevo";
static const uint8_t bnmsg_di_fw_version[] = "0.1";
static const uint8_t bnmsg_di_sw_version[] = "0.1";
static const uint8_t bnmsg_di_model_string[] = "CDP-TZ01B";

static uint8_t peripheral_local_name[] = "HyouRowGan";
static uint64_t peripheral_bdaddr;
static bool peripheral_random_addr;


static bool is_has_next = false;
static bool is_discover = false;

static bool is_devfound = false;

extern TZ10XX_DRIVER_RNG  Driver_RNG;
extern TZ10XX_DRIVER_GPIO Driver_GPIO;

//UUIDs
BLELib_UUID uuid_motion_sens_serv = {0x988ef07959ddcdfb, 0x00050000672711e5, BLELIB_UUID_128};
BLELib_UUID uuid_sens_val_char    = {0x988ef07959ddcdfb, 0x00050001672711e5, BLELIB_UUID_128};
BLELib_UUID uuid_sens_val_desc    = {0x2902, 0, BLELIB_UUID_16};
//Handles
typedef struct {
    BLELib_OppositeService          motion_sens_serv;
    BLELib_OppositeCharacteristic   sens_val_char;
    BLELib_OppositeDescriptor       sens_val_desc;
} HRG_MSS;
static HRG_MSS handle_mss;

/* BLElib unique id. */
enum {
    GATT_UID_GAP_SERVICE = 0,
    GATT_UID_GAP_DEVICE_NAME,
    GATT_UID_GAP_APPEARANCE,
    
    GATT_UID_DI_SERVICE,
    GATT_UID_DI_MANUF_NAME,
    GATT_UID_DI_FW_VERSION,
    GATT_UID_DI_SW_VERSION,
    GATT_UID_DI_MODEL_STRING,
    /* BlueNinja Motion sensor Service */
    GATT_UID_MOTION_SERVICE,
    GATT_UID_MOTION,
    GATT_UID_MOTION_DESC,
};

/* GAP */
const BLELib_Characteristics gap_device_name = {
    GATT_UID_GAP_DEVICE_NAME, 0x2a00, 0, BLELIB_UUID_16,
    BLELIB_PROPERTY_READ,
    BLELIB_PERMISSION_READ | BLELIB_PERMISSION_WRITE,
    bnmsg_gap_device_name, sizeof(bnmsg_gap_device_name),
    NULL, 0
};
const BLELib_Characteristics gap_appearance = {
    GATT_UID_GAP_APPEARANCE, 0x2a01, 0, BLELIB_UUID_16,
    BLELIB_PROPERTY_READ,
    BLELIB_PERMISSION_READ,
    bnmsg_gap_appearance, sizeof(bnmsg_gap_appearance),
    NULL, 0
};
const BLELib_Characteristics *const gap_characteristics[] = { &gap_device_name, &gap_appearance };
const BLELib_Service gap_service = {
    GATT_UID_GAP_SERVICE, 0x1800, 0, BLELIB_UUID_16,
    true, NULL, 0,
    gap_characteristics, 2
};

/* DIS(Device Informatin Service) */
const BLELib_Characteristics di_manuf_name = {
    GATT_UID_DI_MANUF_NAME, 0x2a29, 0, BLELIB_UUID_16,
    BLELIB_PROPERTY_READ,
    BLELIB_PERMISSION_READ,
    bnmsg_di_manufname, sizeof(bnmsg_di_manufname),
    NULL, 0
};
const BLELib_Characteristics di_fw_version = {
    GATT_UID_DI_FW_VERSION, 0x2a26, 0, BLELIB_UUID_16,
    BLELIB_PROPERTY_READ,
    BLELIB_PERMISSION_READ,
    bnmsg_di_fw_version, sizeof(bnmsg_di_fw_version),
    NULL, 0
};
const BLELib_Characteristics di_sw_version = {
    GATT_UID_DI_SW_VERSION, 0x2a28, 0, BLELIB_UUID_16,
    BLELIB_PROPERTY_READ,
    BLELIB_PERMISSION_READ,
    bnmsg_di_sw_version, sizeof(bnmsg_di_sw_version),
    NULL, 0
};
const BLELib_Characteristics di_model_string = {
    GATT_UID_DI_MODEL_STRING, 0x2a24, 0, BLELIB_UUID_16,
    BLELIB_PROPERTY_READ,
    BLELIB_PERMISSION_READ,
    bnmsg_di_model_string, sizeof(bnmsg_di_model_string),
    NULL, 0
};
const BLELib_Characteristics *const di_characteristics[] = {
    &di_manuf_name, &di_fw_version, &di_sw_version, &di_model_string
};
const BLELib_Service di_service = {
    GATT_UID_DI_SERVICE, 0x180a, 0, BLELIB_UUID_16,
    true, NULL, 0,
    di_characteristics, 4
};

/* Service list */
const BLELib_Service *const hrgn_service_list[] = {
    &gap_service, &di_service
};

/*= BLElib Common callback functions =*/
void connectionCompleteCb(const uint8_t status, const bool master, const uint64_t bdaddr, const uint16_t conn_interval)
{
    sprintf(msg, "status=%d, master=%d, bdaddr=%llx conn_interval=%d\r\n", status, master, bdaddr, conn_interval);
    TZ01_console_puts(msg);

    BLELib_requestMtuExchange(40);
}

void connectionUpdateCb(const uint8_t status, const uint16_t conn_interval, const uint16_t conn_latency)
{
    int ret;
    TZ01_console_puts("Run: BLELib_discoverAllPrimaryServices()\r\n");
    ret = BLELib_discoverAllPrimaryServices();  // -> discoverServices()
    if (ret != BLELIB_OK) {
        sprintf(msg, "BLELib_discoverAllPrimaryServices() failed: %d\r\n", ret);
        TZ01_console_puts(msg);
        BLELib_disconnect(peripheral_bdaddr);   // -> disconnectCb()
    }
    Driver_GPIO.WritePin(11, 1);
}

void mtuExchangeResultCb(const uint8_t status, const uint16_t negotiated_mtu_size)
{
    sprintf(msg, "mtuExchangeResultCb: status=%d negotiated_mtu_size=%d\r\n", status, negotiated_mtu_size);
    TZ01_console_puts(msg);
}

void disconnectCb(const uint8_t status, const uint8_t reason)
{
    TZ01_console_puts("disconnected\r\n");
    peripheral_bdaddr = 0;
    peripheral_random_addr = false;
    
    Driver_GPIO.WritePin(11, 0);
}

void advertisingReport(const uint64_t bdaddr, const bool random_addr, const uint8_t type, const uint8_t *const value, const uint8_t value_len)
{
    uint8_t flen, ftype;
    
    if (BLELib_getState() == BLELIB_STATE_SCANNING) {
        if ((type == 0x08) || (type == 0x09)) {
            memset(msg, 0, sizeof(msg));
            flen = sprintf(msg, "bdaddr=%012llx random=%d type=%02x value=", bdaddr, random_addr, type);
            memcpy(&msg[flen], value, value_len);
            flen = strlen(msg);
            msg[flen]     = '\r';
            msg[flen + 1] = '\n';
            
            uint8_t name_len = strlen(peripheral_local_name);
            if (name_len > value_len) {
                name_len = value_len;
            }
            if (memcmp(peripheral_local_name, value, name_len) == 0) {
                //HyouRowGanが見つかった
                peripheral_bdaddr = bdaddr;
                peripheral_random_addr = random_addr;
                is_devfound = true;
            }
        } else {
            sprintf(msg, "bdaddr=%012llx random=%d type=%02x\r\n", bdaddr, random_addr, type);
        }
    }
    TZ01_console_puts(msg);
}

void isrNewEventCb(void)
{
    /* this sample always call BLELib_run() */
}

void isrWakeupCb(void)
{
    /* this callback is not used currently */
}

BLELib_CommonCallbacks tz01_common_callbacks = {
    connectionCompleteCb,
    connectionUpdateCb,
    mtuExchangeResultCb,
    disconnectCb,
    advertisingReport,
    isrNewEventCb,
    isrWakeupCb
};

/*= BLE server callback functions =*/
BLELib_RespForDemand mtuExchangeDemandCb(const uint16_t client_rx_mtu_size, uint16_t *resp_mtu_size)
{
    *resp_mtu_size = 40;
    return BLELIB_DEMAND_ACCEPT;
}

BLELib_ServerCallbacks tz01_server_callbacks = {
    mtuExchangeDemandCb,
    NULL,
    NULL,
    NULL,
    NULL,
    NULL,
    NULL,
    NULL,
};

/*= BLE client callback functions =*/
void discoverServices(const BLELib_ClientStatus status, const BLELib_OppositeService *const service, const BLELib_UUID *uuid, const bool has_next)
{
    int ret;
    sprintf(msg, "service: %d: %p=%04x-%04x UUID=%016llx-%016llx %d\r\n", status, service, service->start_handle, service->end_handle, uuid->uuid_msb, uuid->uuid_lsb, has_next);
    TZ01_console_puts(msg);
    if ((uuid->uuid_lsb == uuid_motion_sens_serv.uuid_lsb) && (uuid->uuid_msb == uuid_motion_sens_serv.uuid_msb)) {
//    if ((uuid->uuid_lsb == 0x180a) && (uuid->uuid_msb == 0x0000)) {
        //HyouRowGan MotionSensorサービスを発見
        BLELib_copyOppositeService(&handle_mss.motion_sens_serv, service);
        is_discover = true;
        TZ01_console_puts("Discoverd!!\r\n");
    }
    if (!has_next) {
        if (is_discover) {
            is_discover = false;
            sprintf(msg, "Run: BLELib_discoverCharacteristics() %04x-%04x\r\n", handle_mss.motion_sens_serv.start_handle, handle_mss.motion_sens_serv.end_handle);
            TZ01_console_puts(msg);
            ret = BLELib_discoverCharacteristics(&handle_mss.motion_sens_serv); /* -> discoverCharacteristics() */
            if (ret != BLELIB_OK) {
                sprintf(msg, "BLELib_discoverCharacteristics() failed: %d\r\n", ret);
                TZ01_console_puts(msg);
                BLELib_disconnect(peripheral_bdaddr);
            }
        } else {
            BLELib_disconnect(peripheral_bdaddr);   // -> disconnectCb()
        }
    }
}

void discoverIncludedServices(const BLELib_ClientStatus status, const BLELib_OppositeService *const service, const BLELib_UUID *uuid, const bool has_next)
{
    
}

void discoverCharacteristics(const BLELib_ClientStatus status, const BLELib_OppositeCharacteristic *const characteristic, const BLELib_UUID *uuid, const bool has_next)
{
    int ret;
    sprintf(msg, "characteristic: %d: %04x UUID=%016llx-%016llx %d\r\n", status,  characteristic->handle, uuid->uuid_msb, uuid->uuid_lsb, has_next);
    TZ01_console_puts(msg);
    if ((uuid->uuid_lsb == uuid_sens_val_char.uuid_lsb) && (uuid->uuid_msb == uuid_sens_val_char.uuid_msb)) {
        //Valueキャラクタリスティックを発見
        BLELib_copyOppositeCharacteristic(&handle_mss.sens_val_char, characteristic);
        is_discover = true;
        TZ01_console_puts("Discoverd!!\r\n");
    }
    
    if (!has_next) {
        if (is_discover) {
            is_discover = false;
            sprintf(msg, "Run: BLELib_discoverDescriptors() %04x\r\n", handle_mss.sens_val_char.handle);
            TZ01_console_puts(msg);
            ret = BLELib_discoverDescriptors(&handle_mss.sens_val_char);
            if (ret != BLELIB_OK) {
                sprintf(msg, "BLELib_discoverDescriptors() failed: %d\r\n", ret);
                TZ01_console_puts(msg);
                BLELib_disconnect(peripheral_bdaddr);
            }
        } else {
            //キャラクタリスティックが見つからなかった
            BLELib_disconnect(peripheral_bdaddr);   // -> disconnectCb()
        }
    }
}

void discoverDescriptors(const BLELib_ClientStatus status, const BLELib_OppositeDescriptor *const descriptor, const BLELib_UUID * uuid, const bool has_next)
{
    int ret;
    
    sprintf(msg, "descriptor: %d: %04x UUID=%016llx-%016llx %d\r\n", status,  descriptor->handle, uuid->uuid_msb, uuid->uuid_lsb, has_next);
    TZ01_console_puts(msg);
    if ((uuid->uuid_msb == uuid_sens_val_desc.uuid_msb) && (uuid->uuid_lsb == uuid_sens_val_desc.uuid_lsb)) {
        //Characteristic Configurationデスクリプタ発見
        BLELib_copyOppositeDescriptor(&handle_mss.sens_val_desc, descriptor);
        is_discover = true;
        TZ01_console_puts("Discoverd!!\r\n");
    }
    
    if (!has_next) {
        if (is_discover) {
            is_discover = false;
            uint8_t val[] = { 0x01, 0x00 }; //Notification enable
            ret = BLELib_writeDescriptor(&handle_mss.sens_val_desc, val, sizeof(val));
            if (ret != BLELIB_OK) {
                sprintf(msg, "BLELib_writeDescriptor() failed: %d\r\n", ret);
                TZ01_console_puts(msg);
                BLELib_disconnect(peripheral_bdaddr);
            }
        } else {
            //デスクリプタが見つからなかった
            BLELib_disconnect(peripheral_bdaddr);
        }
    }
}

void characteristicReadout(const BLELib_ClientStatus status, const BLELib_OppositeCharacteristic *const characteristic, const uint8_t *const value, const uint8_t value_len, const bool has_next)
{
    
}

void characteristicWriteinResponse(const BLELib_ClientStatus status, const BLELib_OppositeCharacteristic *const characteristic)
{
    
}

void characteristicsMultiReadout(const BLELib_ClientStatus status, const uint8_t *const value, const uint8_t value_len)
{
    
}

void descriptorReadout(const BLELib_ClientStatus status, const BLELib_OppositeDescriptor *const descriptor, const uint8_t *const value, const uint8_t value_len, const bool has_next)
{
    
}

void descriptorWriteinResponse(const BLELib_ClientStatus status, const BLELib_OppositeDescriptor *const descriptor)
{
    uint32_t mtu;
    BLELib_getCurrentMtu(&mtu);
    
    sprintf(msg, "%s(): %d mtu=%d\r\n", __func__, status, mtu);
    TZ01_console_puts(msg);
}

void notificationReceived(const BLELib_ClientStatus status, const BLELib_OppositeCharacteristic *const characteristic, const uint8_t *const value, const uint8_t value_len)
{
    int16_t *gx, *gy, *gz;
    int16_t *ax, *ay, *az;
    int16_t *mx, *my, *mz;
    
    sprintf(msg, "Received bytes=%d\r\n", value_len);
    TZ01_console_puts(msg);
    
    if (value_len >= 18) {
        gx = (int16_t *)&value[0];
        gy = (int16_t *)&value[2];
        gz = (int16_t *)&value[4];
        
        ax = (int16_t *)&value[6];
        ay = (int16_t *)&value[8];
        az = (int16_t *)&value[10];
        
        mx = (int16_t *)&value[12];
        my = (int16_t *)&value[14];
        mz = (int16_t *)&value[16];
        
        sprintf(msg, "gyro: %.2f, %.2f, %.2f accel: %.2f, %.2f, %.2f magm: %d, %d, %d\r\n",
            (float)*gx / 16.4, (float)*gy / 16.4, (float)*gz / 16.4, 
            (float)*ax / 2048, (float)*ay / 2048, (float)*az / 2048, 
            *mx, *my, *mz);
        TZ01_console_puts(msg);
    }
}

void indicationReceived(const BLELib_ClientStatus status, const BLELib_OppositeCharacteristic *const characteristic, const uint8_t *const value, const uint8_t value_len)
{
}

BLELib_ClientCallbacks tz01_client_callbacks = {
    discoverServices,
    discoverIncludedServices,
    discoverCharacteristics,
    discoverDescriptors,
    characteristicReadout,
    characteristicWriteinResponse,
    characteristicsMultiReadout,
    descriptorReadout,
    descriptorWriteinResponse,
    notificationReceived,
    indicationReceived
};

int BLE_tz1em_init(void)
{
    if (TZ1EM_STATUS_OK != tz1emInitializeSystem())
        return 1; /* Must not use UART for LOG before twicIfLeIoInitialize. */
    
    return 0;
}

int BLE_init(uint8_t id)
{
    //LED
    Driver_GPIO.Configure(11, GPIO_DIRECTION_OUTPUT_2MA, GPIO_EVENT_DISABLE, NULL); 
    
    /* create random bdaddr */
    uint32_t randval;
    Driver_PMU.SetPowerDomainState(PMU_PD_ENCRYPT, PMU_PD_MODE_ON);
    Driver_RNG.Initialize();
    Driver_RNG.PowerControl(ARM_POWER_FULL);
    Driver_RNG.Read(&randval);
    Driver_RNG.Uninitialize();
    hrgn_bdaddr |= (uint64_t)randval;
    
    if (id > 3) {
        return 1;   //invalid id
    }

    /* initialize BLELib */
//    int ret;
//    ret = BLELib_initialize(hrgn_bdaddr, BLELIB_BAUDRATE_2304, &tz01_common_callbacks, NULL, NULL, NULL);
//
//    return ret;
    return 0;
}

void BLE_term(void)
{
    BLELib_finalize();
}

void BLE_main(void)
{
    static bool is_init = false;
    static bool is_scan = false;
    int ret;
    
    BLELib_State state;
    bool has_event;
    state = BLELib_getState();
    
    switch (state) {
    case BLELIB_STATE_UNINITIALIZED:
        TZ01_console_puts("BLELIB_STATE_UNINITIALIZED\r\n");
        BLELib_initialize(hrgn_bdaddr, BLELIB_BAUDRATE_2304, &tz01_common_callbacks, &tz01_server_callbacks, &tz01_client_callbacks, NULL);
        break;
    case BLELIB_STATE_INITIALIZED:
        if (is_init == false) {
            TZ01_console_puts("BLELIB_STATE_INITIALIZED\r\n");
            BLELib_setLowPowerMode(BLELIB_LOWPOWER_ON);
            if (BLELib_registerService(hrgn_service_list, 1) == BLELIB_OK) {
                is_init = true;
            } else {
                TZ01_console_puts("BLELib_registerService() failed\r\n");
            }
        }
        break;
    case BLELIB_STATE_READY:
        if (is_scan == false) {
            if (peripheral_bdaddr == 0) {
                TZ01_console_puts("BLELIB_STATE_READY\r\n");
                if (BLELib_startScan() == BLELIB_OK) {
                    is_scan = true;
                } else {
                    TZ01_console_puts("BLELib_startScan() failed\r\n");
                }
            }
        }
        break;
    case BLELIB_STATE_SCANNING:
        if ((is_devfound) && (BLELib_hasEvent() == false)) {
            BLELib_connect(peripheral_bdaddr, peripheral_random_addr);    // -> connectionCompleteCb()
            is_devfound = false;
        }
        break;
    case BLELIB_STATE_ONLINE:
        if (is_scan == true) {
            is_scan = false;
        }
        break;
    }
    
    has_event = BLELib_hasEvent();
    if (has_event) {
        ret = BLELib_run();
        if (ret != BLELIB_OK) {
            sprintf(msg, "BLELib_run() ret: %d\r\n", ret);
            TZ01_console_puts(msg);
        }
    }
}
