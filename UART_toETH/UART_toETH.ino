#include <SPI.h> // это нужно для работы Ethernet-шилда
#include <Ethernet.h>
#include <SoftwareSerial.h>

byte mac[] = { 0xAA, 0xAB, 0xAC, 0xAD, 0xAE, 0xAF }; // укажите желаемый MAC
IPAddress ip(192, 168, 0, 155); // укажите желаемый IP
const int port = 7777; // укажите желаемый порт
EthernetServer server(port);

SoftwareSerial uart(8, 9); // сюда подключаем внешний UART (8 - RX, 9 - TX)

String tcpData; //строки для временного хранения сообщений
String uartData; 

void setup() {
  Serial.begin(9600);
  uart.begin(9600);
  Serial.println("UART started");
  Ethernet.begin(mac, ip);
  server.begin();
  Serial.print("Server started at ");
  IPAddress lip = Ethernet.localIP();
  for (byte thisByte = 0; thisByte < 4; thisByte++) {
    Serial.print(lip[thisByte], DEC);
    Serial.print(".");
  }
  Serial.println(":" + (String)port);
}

void loop() {
  EthernetClient client = server.available(); // ожидаем подключения  
  if (client) { // принимаем запрос на подключение TCP клиента
    while (client.connected()) { // пока клиент подключён
    
      // ЕСЛИ ПРИШЛИ ДАННЫЕ ПО UART:
      if (uart.available() > 0) {
        char c = uart.read();
        if (c != '\n') {
          uartData.concat(c); // добавляем прочитанный символ к временной строке
        }
        else {
          Serial.println("UART data received: " + uartData);
          client.println(uartData); // отправляем принятую строку TCP клиенту
          uartData = ""; // обнуляем временную строку
        }
      }
      
      // ЕСЛИ ПРИШЛИ ДАННЫЕ ПО TCP:
      if (client.available()) {
        char c = client.read();
        if (c != '\n') {
          tcpData.concat(c);
        }
        else {
          Serial.println("TCP data received: " + tcpData);
          uart.println(tcpData); // отправляем принятые данные в UART
          tcpData = "";
        }
      }
      
    }    
    client.stop(); // закрываем соединение
  }
}
