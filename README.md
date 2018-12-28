# BME280 Example

This is a direct port of the BME280 sensor API from Bosch: https://github.com/BoschSensortec/BME280_driver

Changes made to the driver are VERY minimal:
 * Increased wait after soft reset - needed on my setup for whatever reason. 2ms wasn't enough time, and I got incorrect readings.
 * Disabled 64-bit compensation by default. I tested 64-bit and floating point (enable by enabling BME280_64BIT_ENABLE or BME280_FLOAT_ENABLE in components/bme280_driver/bme280_defs.h)
 
 There's still a lot that'd be nice to do, but this is functional enough for the project I intend to use it for (environmental monitoring and reporting via MQTT over wifi)
 
# TODO/Nice to have:
 * Better error reporting/handling - for example, a macro to check results and report.
 * Fix the write function to write >1 byte. Currently the driver only ever writes a single byte, so this works and is simple. It'd be nice to fix, though.
 * Better testing - I tested forced mode and normal mode. There is no testing at the API level or, really, any testing at all beyond "It works for me!"
 * Incorporate Kconfig into the options for the BME280 - this would be pretty easy, just some yes/no questions for the above 2 header changes. There's probably more to add, too.
 * Figure out stack size to use, the current is just a guess and probably way too large.
