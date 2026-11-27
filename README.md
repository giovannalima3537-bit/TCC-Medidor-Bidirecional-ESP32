# Protótipo de medidor bidirecional de energia elétrica utilizando ESP32

Código-fonte V4 desenvolvido para trabalho de conclusão de curso em Engenharia Elétrica (2026).

## Estrutura
- `src/main.cpp`: firmware C++ e interface web embarcada.
- `platformio.ini`: configuração do projeto PlatformIO e dependências.

## Funcionalidades
- Aquisição analógica: GPIO34 (corrente) e GPIO35 (tensão).
- Diagnóstico de bancada e tratamento baseado em mediana e MAD.
- Ajuste senoidal da componente de 60 Hz em bancada.
- Simulação matemática com comparação MAD e DFT.
- Interface OLED I²C e interface web local via Wi-Fi/WebSocket.

## Ambiente
ESP32 DOIT DEVKIT V1, PlatformIO e framework Arduino. Abra a pasta do projeto no Visual Studio Code com PlatformIO. As dependências estão declaradas em `platformio.ini`. Os valores de Wi-Fi são exemplos; configure credenciais locais sem publicá-las.

## Limitações e segurança
Versão de bancada, sem conexão à rede elétrica. O firmware mantém a apresentação das grandezas físicas bloqueada até a validação elétrica e de fase. Os resultados de simulação não são medições reais. Não conecte esta montagem experimental à rede AC. O protótipo não substitui medidores homologados para faturamento.
