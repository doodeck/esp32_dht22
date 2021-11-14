/* test_dht.h */

/* careful NOT WORKING yet */

#ifndef __TEST_DHT_H__
#define __TEST_DHT_H__

/* test the transmission between esp32 and dht22 without interpreting it */
/* connect DHT22 pin 2 to any esp32 pin, which can be pulled up */

void test_dht_init();
/* returns 0 on success */
int test_dht(int pin);

#endif /* __TEST_DHT_H__ */
