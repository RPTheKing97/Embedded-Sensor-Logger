# Telemetry Format

The firmware sends one UART line every 100 ms.

## Example

```text
TS:12345,V:3,AX_mg:12,AY_mg:-5,AZ_mg:1001,GX_cdps:2,GY_cdps:-3,GZ_cdps:1,T_cC:2451,P_chPa:101325,ERR:0,RET:0,S:1234
```

## Fields

| Field | Meaning | Unit |
|---|---|---|
| `TS` | HAL timestamp | ms |
| `V` | sensor validity mask | bitmask |
| `AX_mg` | filtered acceleration X | milli-g |
| `AY_mg` | filtered acceleration Y | milli-g |
| `AZ_mg` | filtered acceleration Z | milli-g |
| `GX_cdps` | gyro X | centi-degrees/sec |
| `GY_cdps` | gyro Y | centi-degrees/sec |
| `GZ_cdps` | gyro Z | centi-degrees/sec |
| `T_cC` | temperature | centi-degrees Celsius |
| `P_chPa` | pressure | centi-hPa |
| `ERR` | communication/fault errors | count |
| `RET` | I2C retries | count |
| `S` | samples taken | count |

## Validity Mask

| V | Meaning |
|---|---|
| `3` | MPU6050 + BMP280 OK |
| `1` | MPU6050 OK, BMP280 missing/fault |
| `2` | BMP280 OK, MPU6050 missing/fault |
| `0` | Both sensors missing/fault |
