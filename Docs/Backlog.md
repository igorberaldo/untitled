# Descrição do firmware

## O sistema tem 3 firmwares

* Bluetooth(NRF52832): Configuração, Comunic. App, devices seguros.
* Wifi(ESP8266): Armazena leitur. sensores, envia para servidor, config. via servidor.
* Microcontrolador(STM32): Fimrware Principal, controla os outros módulos, lê sensores, nível de bat, carregamento, CATm.

## Modos de Operação

### 1. Rotulação [Jaime]

* [ ] 1.1 O microcontrolador acorda o Bluetooth e espera a conexão com o app;  
* [ ] 1.2 O microcontrolador aguarda o start do app;  
* [ ] 1.3 Microcontrolador coleta X leituras e enviar para o APP;  
* [ ] 1.4 O microcontrolador aguarda o stop do app.

### 2. Coletando (Modo Seguro | Principal)  [Jaime]

* [ ] 2.1 Em repouso(baixo consumo de bateria, somente o acelerômetro ligado em baixo consumo) até detectar movimento;  
* [ ] 2.2 Acelerômetro detectou movimento acorda o microcontrolador(STM32);  
* [ ] 2.3 Microcontrolador coleta X leituras, até chegar no limite de armazenamento;  
* [ ] 2.4 Microcontrolador acorda o módulo WIFI e descarrega as leituras, o WIFI armazena e volta a dormir;  
* [ ] 2.5 O WIFI quando estiver no limite do armazenamento, conecta na rede, descarrega as leituras, verifica se tem alguma mensagem para ele e volta a dormir.  

### 3. Alerta (Modo inseguro | Fuga)  [Gustavo]

* [ ] 3.1 Após um tempo X acabar o Microcontrolador acorda o WIFI e pergunta se está seguro. Se estiver seguro ele volta a dormir;  
* [ ] 3.2 Se o WIFI responder que não está seguro, o Microcontrolador acorda o Bluetooth e pergunta se está seguro, se estiver seguro volta a dormir;  
* [ ] 3.3 Se o Bluetooth responder que não está seguro, o Microcontrolador acorda o GPS e verifica se está seguro, se estiver seguro volta a dormir;  
* [ ] 3.4 Se o GPS informar que não está seguro, o Microcontrolador acorda o CATm, conecta no servidor, informa que está em modo de alerta, verifica se tem alguma mensagem, desliga tudo e vai dormir por 5min;  
* [ ] 3.5 O Microcontrolador ao acordar dos 5m, faz toda a verificação se está seguro, se estiver volta ao estado coletando;  
* [ ] 3.6 Se não estiver em modo seguro, repete o passo 3.4.

### 4. Configuração (Setup Inicial)  [Gustavo]

* [ ] 4.1 O Microcontrolador detecta que o botão foi pressionado por X segundos, começa a piscar o led 3x por segundo, acorda o módulo Bluetooth e ativa o módulo de emparelhamento no módulo bluetooth;  
* [ ] 4.2 O Bluetooth aguarda um dispositivo conectar com a chave XXXXXXXXX, conecta ao dispositivo;  
* [ ] 4.3 O Bluetooth envia as configurações para o dispositivo conectado;  
* [ ] 4.4 O Bluetooth recebe novas configurações;  
* [ ] 4.5 O Microcontrolador pergunta ao Bluetooth por novas configurações, recebe e salva;  
* [ ] 4.6 O Microcontrolador pergunta ao Bluetooth se a configuração acabou, se sim, vai para o modo Coletando/Seguro.  
