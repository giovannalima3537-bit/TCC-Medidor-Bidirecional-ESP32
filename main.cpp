
#include <Arduino.h>
#include <WiFi.h>
#include <Wire.h>
#include <U8g2lib.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <math.h>

// =====================================================
// TCC - MEDIDOR BIDIRECIONAL DE ENERGIA
// V4 - Diagnostico real com ajuste de 60 Hz + simulacao V3 preservada
// ESP32 DOIT DEVKIT V1
// GPIO34: SCT-013-000
// GPIO35: ZMPT101B
// OLED: SDA 21 / SCL 22
// NAO CONECTAR A REDE AC NESTA FASE
// =====================================================

const char *WIFI_SSID = "SUA_REDE";
const char *WIFI_SENHA = "SUA_SENHA";
const char *AP_SSID = "TCC-Medidor";
const char *AP_SENHA = "AltereEstaSenha2026";

const uint8_t PINO_I = 34;
const uint8_t PINO_V = 35;
const uint8_t SDA_OLED = 21;
const uint8_t SCL_OLED = 22;

const uint16_t AMOSTRAS = 600;
const uint8_t JANELAS_RUIDO = 21;
const uint8_t JANELAS_CONFIRMACAO = 3;

const double ALFA = 0.25;
const double MARGEM_MIN_ENTRADA_V = 0.004;
const double MARGEM_MIN_SAIDA_V = 0.002;
const double MULTIPLO_MAD_ENTRADA = 3.0;
const double MULTIPLO_MAD_SAIDA = 1.5;

// Calibracao eletrica real ainda pendente.
const bool MEDICAO_VALIDADA = false;
const double FATOR_V = 0.0;
const double FATOR_I = 0.0;
const double AJUSTE_FASE = 1.0;

U8G2_SSD1306_128X64_NONAME_F_HW_I2C oled(
    U8G2_R0, U8X8_PIN_NONE);

AsyncWebServer servidor(80);
AsyncWebSocket ws("/ws");

bool modoAP = false;

// =====================================================
// PROCESSAMENTO MATEMATICO
// =====================================================

struct Estatisticas {
  uint32_t n = 0;
  double mediaI = 0.0;
  double mediaV = 0.0;
  double m2I = 0.0;
  double m2V = 0.0;
  double cov = 0.0;

  void adicionar(double i, double v) {
    ++n;

    double dI = i - mediaI;
    mediaI += dI / n;
    m2I += dI * (i - mediaI);

    double dV = v - mediaV;
    mediaV += dV / n;
    m2V += dV * (v - mediaV);

    cov += dI * (v - mediaV);
  }

  double rmsI() const {
    return n ? sqrt(fmax(0.0, m2I / n)) : 0.0;
  }

  double rmsV() const {
    return n ? sqrt(fmax(0.0, m2V / n)) : 0.0;
  }

  double potenciaADC() const {
    return n ? cov / n : 0.0;
  }
};

struct Resultado {
  double offsetI = 0.0;
  double offsetV = 0.0;
  double brutoI = 0.0;
  double filtradoI = 0.0;
  double acV = 0.0;

  double vrms = 0.0;
  double irms = 0.0;
  double p = 0.0;
  double s = 0.0;
  double fp = 0.0;

  double energiaImport = 0.0;
  double energiaExport = 0.0;

  const char *fluxo = "CALIBRACAO PENDENTE";
};

Resultado leitura;

double janelaRuido[JANELAS_RUIDO] = {0};
double ruidoBase = 0;
double dispersaoRuido = 0;
double limiarEntrada = 0;
double limiarSaida = 0;
double rmsFiltrado = 0;

uint8_t janelas = 0;
uint8_t seguidasAltas = 0;
uint8_t seguidasBaixas = 0;

bool ruidoCalibrado = false;
bool acimaLimiar = false;

unsigned long instanteEnergia = 0;

// =====================================================
// MEDIANA E FILTRO ROBUSTO
// =====================================================

double mediana(const double *dados, uint8_t n) {
  if (n == 0) return 0;

  double ordenados[JANELAS_RUIDO];

  for (uint8_t i = 0; i < n; i++)
    ordenados[i] = dados[i];

  for (uint8_t i = 1; i < n; i++) {
    double chave = ordenados[i];
    int j = (int)i - 1;

    while (j >= 0 && ordenados[j] > chave) {
      ordenados[j + 1] = ordenados[j];
      j--;
    }

    ordenados[j + 1] = chave;
  }

  if (n % 2)
    return ordenados[n / 2];

  return (ordenados[n / 2 - 1] +
          ordenados[n / 2]) / 2.0;
}

void terminarCalibracaoRuido() {
  ruidoBase = mediana(janelaRuido, JANELAS_RUIDO);

  double desvios[JANELAS_RUIDO];

  for (uint8_t i = 0; i < JANELAS_RUIDO; i++) {
    desvios[i] = fabs(janelaRuido[i] - ruidoBase);
  }

  dispersaoRuido =
      1.4826 * mediana(desvios, JANELAS_RUIDO);

  limiarEntrada = ruidoBase + fmax(
      MARGEM_MIN_ENTRADA_V,
      MULTIPLO_MAD_ENTRADA * dispersaoRuido);

  limiarSaida = ruidoBase + fmax(
      MARGEM_MIN_SAIDA_V,
      MULTIPLO_MAD_SAIDA * dispersaoRuido);

  acimaLimiar = false;
  seguidasAltas = 0;
  seguidasBaixas = 0;
  rmsFiltrado = 0;
  ruidoCalibrado = true;

  Serial.printf(
      "Ruido base: %.4f V | Dispersao: %.4f V\n",
      ruidoBase, dispersaoRuido);

  Serial.printf(
      "Limiar entrada: %.4f V | Saida: %.4f V\n",
      limiarEntrada, limiarSaida);

  Serial.println(
      "Calibracao matematica concluida.");
}

void aplicarFiltro(double rmsBruto) {
  if (!acimaLimiar) {
    if (rmsBruto > limiarEntrada) {
      if (++seguidasAltas >= JANELAS_CONFIRMACAO) {
        acimaLimiar = true;
        seguidasBaixas = 0;
      }
    } else {
      seguidasAltas = 0;
    }
  } else {
    if (rmsBruto < limiarSaida) {
      if (++seguidasBaixas >= JANELAS_CONFIRMACAO) {
        acimaLimiar = false;
        seguidasAltas = 0;
      }
    } else {
      seguidasBaixas = 0;
    }
  }

  if (acimaLimiar) {
    double compensado = sqrt(fmax(
        0.0,
        rmsBruto * rmsBruto - ruidoBase * ruidoBase));

    rmsFiltrado =
        ALFA * compensado +
        (1.0 - ALFA) * rmsFiltrado;
  } else {
    rmsFiltrado = 0;
  }

  leitura.filtradoI = rmsFiltrado;
}

bool dadosCalibrados() {
  // V4 de bancada: bloqueio intencional ate validacao eletrica e de fase.
  return false;
}

String ipDoESP() {
  return modoAP
      ? WiFi.softAPIP().toString()
      : WiFi.localIP().toString();
}

// =====================================================
// JSON - DIAGNOSTICO REAL
// =====================================================

// Ajuste senoidal por minimos quadrados com timestamps reais.
// Equivalente a projecao DFT em amostragem uniforme coerente.
struct Ajuste60 {
  double m[3][4] = {};
  void add(double x, uint32_t us) {
    double a = 6.283185307179586 * 60.0 * us / 1000000.0;
    double b[3] = {1.0, cos(a), sin(a)};
    for(int i=0;i<3;i++) {
      for(int j=0;j<3;j++) m[i][j]+=b[i]*b[j];
      m[i][3]+=b[i]*x;
    }
  }
  bool solve(double &dc,double &c,double &q) {
    for(int i=0;i<3;i++) {
      int k=i;
      for(int j=i+1;j<3;j++) if(fabs(m[j][i])>fabs(m[k][i])) k=j;
      if(fabs(m[k][i])<1e-9) return false;
      for(int j=0;j<4;j++) {double z=m[i][j];m[i][j]=m[k][j];m[k][j]=z;}
      double z=m[i][i]; for(int j=i;j<4;j++) m[i][j]/=z;
      for(int k=0;k<3;k++) if(k!=i) {z=m[k][i];for(int j=i;j<4;j++)m[k][j]-=z*m[i][j];}
    }
    dc=m[0][3];c=m[1][3];q=m[2][3];return true;
  }
};
const unsigned N_GRAF=60;
float grafI[N_GRAF],grafV[N_GRAF];
uint32_t grafTI[N_GRAF],grafTV[N_GRAF];
double realI60=0,realV60=0,baseI60=0,baseV60=0,fsReal=0;
double refI60[JANELAS_RUIDO],refV60[JANELAS_RUIDO];
unsigned atrasos=0,extI=0,extV=0;
bool ajusteOk=false;
String criarJSON() {
  String j;
  j.reserve(6500);

  j = "{";

  j += "\"calibrado\":";
  j += dadosCalibrados() ? "true" : "false";

  j += ",\"ruidoCalibrado\":";
  j += ruidoCalibrado ? "true" : "false";

  j += ",\"offsetI\":" + String(leitura.offsetI, 3);
  j += ",\"offsetV\":" + String(leitura.offsetV, 3);
  j += ",\"rmsIbruto\":" + String(leitura.brutoI, 4);
  j += ",\"rmsIfiltrado\":" + String(leitura.filtradoI, 4);
  j += ",\"rmsVadc\":" + String(leitura.acV, 4);
  j += ",\"ruidoBase\":" + String(ruidoBase, 4);
  j += ",\"dispersaoRuido\":" + String(dispersaoRuido, 4);
  j += ",\"limiarEntrada\":" + String(limiarEntrada, 4);
  j += ",\"limiarSaida\":" + String(limiarSaida, 4);

  j += ",\"acimaLimiar\":";
  j += acimaLimiar ? "true" : "false";

  j += ",\"janelasCalibradas\":" + String(janelas);
  j += ",\"totalJanelas\":" + String(JANELAS_RUIDO);

  j += ",\"tensao\":";
  j += dadosCalibrados() ? String(leitura.vrms, 1) : "null";

  j += ",\"corrente\":";
  j += dadosCalibrados() ? String(leitura.irms, 3) : "null";

  j += ",\"potencia\":";
  j += dadosCalibrados() ? String(leitura.p, 1) : "null";

  j += ",\"aparente\":";
  j += dadosCalibrados() ? String(leitura.s, 1) : "null";

  j += ",\"fp\":";
  j += dadosCalibrados() && leitura.s > 0
      ? String(leitura.fp, 3) : "null";

  j += ",\"energiaImport\":";
  j += dadosCalibrados()
      ? String(leitura.energiaImport, 6) : "null";

  j += ",\"energiaExport\":";
  j += dadosCalibrados()
      ? String(leitura.energiaExport, 6) : "null";

  j += ",\"fluxo\":\"" + String(leitura.fluxo) + "\"";
  j += ",\"i60\":" + String(realI60*1000,3);
  j += ",\"v60\":" + String(realV60*1000,3);
  j += ",\"baseI60\":"; j += ruidoCalibrado ? String(baseI60*1000,3) : "null";
  j += ",\"baseV60\":"; j += ruidoCalibrado ? String(baseV60*1000,3) : "null";
  j += ",\"fsReal\":" + String(fsReal,1);
  j += ",\"atrasos\":" + String(atrasos);
  j += ",\"extI\":" + String(extI);
  j += ",\"extV\":" + String(extV);
  j += ",\"ajusteOk\":"; j += ajusteOk ? "true" : "false";
  j += ",\"ondasReais\":[";
  for(unsigned k=0;k<N_GRAF;k++) {
    if(k) j += ",";
    j += "["+String(grafTI[k])+","+String((grafI[k]-leitura.offsetI)*1000,2)+","+String(grafTV[k])+","+String((grafV[k]-leitura.offsetV)*1000,2)+"]";
  }
  j += "]";
  j += "}";

  return j;
}

// =====================================================
// SIMULACAO MATEMATICA INDEPENDENTE
// =====================================================

String calcularSimulacao(int caso, bool ruido) {
  const double PI_SIM = 3.141592653589793;
  const double FS = 1200.0;
  const double FREQ = 60.0;

  // Fator nominal ilustrativo, nao calibracao real.
  const double FATOR_SIM = 2000.0 / 22.0;

  double v = 127.0;
  double i = 5.0;
  double fase = 0;

  switch (caso) {
    case 1:
      fase = PI_SIM;
      break;

    case 2:
      fase = PI_SIM / 3.0;
      break;

    case 3:
      i = 0.0;
      break;

    case 4:
      i = 0.2;
      break;

    case 5:
      fase = PI_SIM / 2.0;
      break;

    default:
      break;
  }

  Estatisticas ideal;
  Estatisticas adc;
  // DFT de uma frequencia: 600 amostras a 1200 Hz = 30 ciclos.
  double ic=0, is=0, vc=0, vs=0;

  for (uint16_t k = 0; k < AMOSTRAS; k++) {
    double t = k / FS;
    double a = 2.0 * PI_SIM * FREQ * t;

    double sinalV = sqrt(2.0) * v * sin(a);
    double sinalI = sqrt(2.0) * i * sin(a - fase);

    double n = ruido
        ? sqrt(2.0) * 0.009 *
          sin(2.0 * PI_SIM * 420.0 * t)
        : 0.0;

    ideal.adicionar(sinalI, sinalV);

    const double entradaI = 1.66 + sinalI / FATOR_SIM + n;
    adc.adicionar(entradaI, sinalV);
    ic += entradaI * cos(a);
    is += entradaI * sin(a);
    vc += sinalV * cos(a);
    vs += sinalV * sin(a);
  }

  double vrms = ideal.rmsV();
  double irms = ideal.rmsI();
  double potencia = ideal.potenciaADC();

  if (fabs(potencia) < 0.00000001)
    potencia = 0;

  double aparente = vrms * irms;

  double fp = aparente > 0
      ? fmin(1.0, fabs(potencia / aparente))
      : 0;

  double bruto = adc.rmsI();

  double ref[JANELAS_RUIDO];
  double desvios[JANELAS_RUIDO];

  for (uint8_t k = 0; k < JANELAS_RUIDO; k++) {
    ref[k] = ruido
        ? 0.0085 + 0.001 * k / (JANELAS_RUIDO - 1)
        : 0.0;
  }

  double base = mediana(ref, JANELAS_RUIDO);

  for (uint8_t k = 0; k < JANELAS_RUIDO; k++)
    desvios[k] = fabs(ref[k] - base);

  double dispersao =
      1.4826 * mediana(desvios, JANELAS_RUIDO);

  double entrada = base + fmax(
      MARGEM_MIN_ENTRADA_V,
      MULTIPLO_MAD_ENTRADA * dispersao);

  double saida = base + fmax(
      MARGEM_MIN_SAIDA_V,
      MULTIPLO_MAD_SAIDA * dispersao);

  bool detectado = false;

  uint8_t altas = 0;
  uint8_t baixas = 0;

  double filtrado = 0;

  for (uint8_t k = 0; k < 8; k++) {
    if (!detectado) {
      if (bruto > entrada) {
        if (++altas >= JANELAS_CONFIRMACAO) {
          detectado = true;
          baixas = 0;
        }
      } else {
        altas = 0;
      }
    } else {
      if (bruto < saida) {
        if (++baixas >= JANELAS_CONFIRMACAO) {
          detectado = false;
          altas = 0;
        }
      } else {
        baixas = 0;
      }
    }

    if (detectado) {
      double compensado = sqrt(fmax(
          0.0, bruto * bruto - base * base));

      filtrado =
          ALFA * compensado +
          (1.0 - ALFA) * filtrado;
    } else {
      filtrado = 0;
    }
  }

  // Coeficientes RMS da fundamental; o offset DC e ortogonal
  // nesta janela sintetica de ciclos inteiros.
  const double escalaDFT = sqrt(2.0) / AMOSTRAS;
  ic *= escalaDFT * FATOR_SIM;
  is *= escalaDFT * FATOR_SIM;
  vc *= escalaDFT;
  vs *= escalaDFT;
  double iDFT = hypot(ic, is);
  double vDFT = hypot(vc, vs);
  double pDFT = ic*vc + is*vs;
  if (iDFT < 1e-9) iDFT = 0;
  if (fabs(pDFT) < 1e-8) pDFT = 0;
  double sDFT = vDFT * iDFT;
  double fpDFT = sDFT > 1e-9 ? fmin(1.0, fabs(pDFT/sDFT)) : 0;
  double iMAD = filtrado * FATOR_SIM;
  double erroI = fabs(iDFT - irms);
  double erroP = fabs(pDFT - potencia);
  const char *fluxoDFT = pDFT > 5 ? "IMPORTANDO" :
      pDFT < -5 ? "EXPORTANDO" : "SEM FLUXO ATIVO";

  const char *fluxo =
      potencia > 5 ? "IMPORTANDO" :
      potencia < -5 ? "EXPORTANDO" :
      "SEM FLUXO ATIVO";

  // Cada passo equivale a 60 segundos simulados.
  double eImport =
      fmax(0.0, potencia) * 60.0 / 3600000.0;

  double eExport =
      fmax(0.0, -potencia) * 60.0 / 3600000.0;

  String j;
  j.reserve(1300);

  j = "{\"simulado\":true";

  j += ",\"v\":" + String(vrms, 4);
  j += ",\"i\":" + String(irms, 4);
  j += ",\"p\":" + String(potencia, 4);
  j += ",\"s\":" + String(aparente, 4);

  j += ",\"fp\":";
  j += aparente > 0.000001
      ? String(fp, 4) : "null";

  j += ",\"fase\":" + String(fase, 8);
  j += ",\"bruto\":" + String(bruto, 8);
  j += ",\"filtrado\":" + String(filtrado, 8);
  j += ",\"base\":" + String(base, 8);
  j += ",\"dispersao\":" + String(dispersao, 8);
  j += ",\"entrada\":" + String(entrada, 8);
  j += ",\"saida\":" + String(saida, 8);

  j += ",\"detectado\":";
  j += detectado ? "true" : "false";

  j += ",\"iMAD\":" + String(iMAD, 8);
  j += ",\"iDFT\":" + String(iDFT, 8);
  j += ",\"vDFT\":" + String(vDFT, 8);
  j += ",\"pDFT\":" + String(pDFT, 8);
  j += ",\"sDFT\":" + String(sDFT, 8);
  j += ",\"fpDFT\":";
  j += sDFT > 1e-9 ? String(fpDFT, 8) : "null";
  j += ",\"erroI\":" + String(erroI, 8);
  j += ",\"erroP\":" + String(erroP, 8);
  j += ",\"erroIpct\":";
  j += irms > 1e-9 ? String(100*erroI/irms, 8) : "null";
  j += ",\"erroPpct\":";
  j += fabs(potencia) > 1e-9 ? String(100*erroP/fabs(potencia), 8) : "null";
  j += ",\"fluxoDFT\":\"" + String(fluxoDFT) + "\"";
  j += ",\"eImport\":" + String(eImport, 10);
  j += ",\"eExport\":" + String(eExport, 10);

  j += ",\"fluxo\":\"" + String(fluxo) + "\"";
  j += "}";

  return j;
}

// =====================================================
// DASHBOARD WEB
// =====================================================

const char PAGINA[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html lang="pt-BR">
<head>
<meta charset="UTF-8">
<meta name="viewport"
content="width=device-width,initial-scale=1">

<title>TCC - Medidor Bidirecional</title>

<style>
*{box-sizing:border-box}
body{
  margin:0;
  background:#eef3f8;
  color:#18253c;
  font-family:Arial,sans-serif
}
header{
  background:#102a48;
  color:white;
  padding:24px 16px;
  text-align:center
}
header h1{margin:0;font-size:25px}
header p{color:#cad9e9;margin:10px 0 0}
main{max-width:1100px;margin:auto;padding:20px}
nav{display:flex;gap:10px;margin-bottom:18px}
button,select{
  font:inherit;
  border:1px solid #cbd5e1;
  border-radius:8px;
  padding:12px;
  background:white;
  cursor:pointer
}
nav button{flex:1}
button.ativo{
  background:#087b99;
  color:white;
  border-color:#087b99
}
.grid{
  display:grid;
  grid-template-columns:repeat(auto-fit,minmax(170px,1fr));
  gap:12px
}
.card{
  background:white;
  padding:18px;
  border-radius:12px;
  box-shadow:0 2px 10px #18345412
}
.card h3{
  color:#63758a;
  margin:0 0 12px;
  font-size:14px;
  font-weight:normal
}
.value{
  color:#087b99;
  font-size:23px;
  font-weight:bold;
  overflow-wrap:anywhere
}
h2{font-size:18px;margin:25px 0 12px}
.note{
  padding:14px;
  background:#fffaf0;
  border-left:4px solid #d99a20;
  border-radius:6px;
  margin-bottom:15px
}
.controls{
  display:flex;
  align-items:center;
  flex-wrap:wrap;
  gap:12px
}
small{color:#60758a}
canvas{
  width:100%;
  height:220px;
  display:block
}
.legend{
  display:flex;
  justify-content:center;
  gap:25px;
  margin:12px
}
.azul{color:#2476c5}
.laranja{color:#d78a18}
.hidden{display:none}
footer{
  text-align:center;
  padding:25px;
  color:#60758a;
  font-size:13px
}
</style>
</head>

<body>

<header>
<h1>TCC - Medidor Bidirecional</h1>
<p>ESP32 | SCT-013-000 | ZMPT101B | OLED</p>
</header>

<main>

<nav>
<button id="btnReal" class="ativo"
onclick="mudarModo(false)">
Modo real
</button>

<button id="btnSim"
onclick="mudarModo(true)">
Modo simulacao
</button>
</nav>

<!-- MODO REAL -->

<section id="real">

<div class="note">
<b>Estado:</b>
<span id="estadoReal">Conectando...</span>
<br>
<small>
Grandezas eletricas bloqueadas ate a calibracao.
</small>
</div>

<h2>Grandezas eletricas reais</h2>

<div class="grid" id="gridReal"></div>

<h2>Diagnostico de bancada</h2>

<div class="grid" id="gridDiagnostico"></div>
<h2>V4 - Analise real da componente de 60 Hz</h2>
<div class="grid" id="gridReal60"></div>
<p><small>Ajuste senoidal a 60 Hz com timestamps individuais; valores em mV no ADC, sem conversao em amperes ou volts da rede. Baselines sao medianas de 21 janelas sem carga, nao limiares validados. Extremos ADC sao indicadores de possivel saturacao, nao garantia de faixa linear.</small></p>
<h2>Sinais reais dos ADCs - primeiras 60 amostras</h2>
<div class="card"><canvas id="ondasReais"></canvas><div class="legend"><b class="azul">GPIO35 - tensao</b><b class="laranja">GPIO34 - corrente</b></div><small>Escala comum em mV, offset removido. Tempo real em ms.</small></div>

<p>
<small>
Os valores de diagnostico representam tensoes
nas entradas analogicas do ESP32.
</small>
</p>

<p>
Conexao:
<b id="conexao">Aguardando</b>
</p>

</section>

<!-- MODO SIMULACAO -->

<section id="sim" class="hidden">

<div class="note">
<b>SIMULACAO MATEMATICA</b><br>
Os valores desta pagina nao sao medicoes reais.
A simulacao nao altera os sensores nem sua calibracao.
</div>

<div class="card">

<div class="controls">

<label>Cenario:</label>

<select id="caso">
<option value="0">Importacao - 5 A</option>
<option value="1">Exportacao - 5 A</option>
<option value="2">Carga indutiva - FP 0,5</option>
<option value="3">Sem carga</option>
<option value="4">Corrente pequena - 0,2 A</option>
<option value="5">Quadratura - FP 0</option>
</select>

<label>
<input type="checkbox" id="ruido" checked>
Ruido sintetico
</label>

<button id="iniciar" onclick="alternarExecucao()">
Iniciar
</button>

<button onclick="zerarEnergia()">
Zerar energia
</button>

</div>

<p id="tempoSim">Tempo simulado: 0 s</p>

<small>
Cada passo de execucao equivale a 60 segundos
simulados. Os acumuladores sao exclusivos
desta aba do navegador.
</small>

</div>

<h2>Grandezas eletricas simuladas - referencia ideal</h2>

<div class="grid" id="gridSim"></div>

<h2>V3 - Comparativo MAD x DFT de 60 Hz (simulacao preservada)</h2>
<div class="grid" id="gridDFT"></div>
<p><small>DFT aplicada ao sinal sintetico com offset e ruido de 420 Hz. Tensao sintetica ideal. MAD: saida apos oito janelas. DFT: fundamental em janela de 30 ciclos. Erro percentual indefinido quando a referencia e zero. A DFT nao possui limiar de deteccao validado e nao mede harmonicas. Os acumuladores acima continuam sendo de referencia ideal.</small></p>
<h2>Formas de onda - 60 Hz</h2>

<div class="card">

<canvas id="ondas"></canvas>

<div class="legend">
<b class="azul">Tensao (V)</b>
<b class="laranja">Corrente (A)</b>
</div>

<small>
Sinais senoidais ideais em escalas verticais
independentes.
</small>

</div>

<h2>Diagnostico do filtro matematico</h2>

<div class="grid" id="gridFiltro"></div>

<p>
<small>
Ruido sintetico de 420 Hz e 9 mV RMS,
quando habilitado. A potencia e a energia
sao calculadas a partir dos sinais ideais,
independentemente do filtro.
</small>
</p>

</section>

</main>

<footer>
Projeto de TCC - Medidor Bidirecional de Energia
</footer>

<script>

const $ = id => document.getElementById(id);

function cards(id, lista) {
  $(id).innerHTML = lista.map(
    x => '<div class="card"><h3>' +
    x[1] + '</h3><div class="value" id="' +
    x[0] + '">--</div></div>'
  ).join('');
}

function fmt(v,n,u='') {
  if(v===null || v===undefined)
    return 'A calibrar';

  return Number(v).toLocaleString('pt-BR',{
    minimumFractionDigits:n,
    maximumFractionDigits:n
  }) + (u ? ' '+u : '');
}

function set(id,v) {
  $(id).textContent=v;
}

cards('gridReal',[
  ['tensao','Tensao RMS'],
  ['corrente','Corrente RMS'],
  ['potencia','Potencia ativa'],
  ['aparente','Potencia aparente'],
  ['fp','Fator de potencia'],
  ['fluxo','Sentido'],
  ['energiaImport','Energia importada'],
  ['energiaExport','Energia exportada']
]);

cards('gridDiagnostico',[
  ['offsetI','Offset de corrente'],
  ['rmsIbruto','RMS corrente bruto'],
  ['rmsIfiltrado','RMS corrente filtrado'],
  ['offsetV','Offset de tensao'],
  ['rmsVadc','RMS tensao no ADC'],
  ['ruidoBase','Ruido-base'],
  ['dispersaoRuido','Dispersao do ruido'],
  ['limiarEntrada','Limiar de entrada'],
  ['limiarSaida','Limiar de saida'],
  ['acimaLimiar','Estado do sinal']
]);

cards('gridReal60',[
 ['rI60','GPIO34 fundamental 60 Hz'],['rV60','GPIO35 fundamental 60 Hz'],
 ['rBI','Base 60 Hz - GPIO34'],['rBV','Base 60 Hz - GPIO35'],
 ['rFS','Taxa efetiva por canal'],['rLate','Atrasos maiores que 1 periodo'],
 ['rEI','Extremos ADC GPIO34'],['rEV','Extremos ADC GPIO35'],['rOK','Ajuste matematico']
]);
function ondasFisicas(d){
 const cv=$('ondasReais'),w=cv.clientWidth,h=cv.clientHeight;
 if(!w||!h||!d.ondasReais||!d.ondasReais.length)return;
 const z=window.devicePixelRatio||1;cv.width=w*z;cv.height=h*z;
 const ctx=cv.getContext('2d');ctx.scale(z,z);
 const a=d.ondasReais, maxT=Math.max(1,...a.map(x=>Math.max(x[0],x[2]))),
 maxY=Math.max(1,...a.map(x=>Math.max(Math.abs(x[1]),Math.abs(x[3]))));
 const left=55,right=w-15,mid=h/2,amp=h/2-25;
 ctx.font='11px Arial';ctx.fillStyle='#60758a';ctx.strokeStyle='#d8e1eb';
 for(let k=0;k<=4;k++){let x=left+(right-left)*k/4;ctx.beginPath();ctx.moveTo(x,15);ctx.lineTo(x,h-22);ctx.stroke();ctx.fillText((maxT*k/4000).toFixed(1),x-10,h-5);}
 ctx.beginPath();ctx.moveTo(left,mid);ctx.lineTo(right,mid);ctx.stroke();
 ctx.fillText('+'+maxY.toFixed(1),2,25);ctx.fillText('0 mV',2,mid);ctx.fillText('-'+maxY.toFixed(1),2,h-25);
 for(const [ti,yi,col] of [[0,1,'#d78a18'],[2,3,'#2476c5']]){ctx.beginPath();ctx.strokeStyle=col;ctx.lineWidth=1.5;a.forEach((x,k)=>{let px=left+(right-left)*x[ti]/maxT,py=mid-amp*x[yi]/maxY;if(k)ctx.lineTo(px,py);else ctx.moveTo(px,py);});ctx.stroke();}
}
cards('gridSim',[
  ['sv','Tensao RMS'],
  ['si','Corrente RMS'],
  ['sp','Potencia ativa'],
  ['ss','Potencia aparente'],
  ['sfp','Fator de potencia'],
  ['sfluxo','Sentido'],
  ['sei','Energia importada'],
  ['see','Energia exportada']
]);

cards('gridDFT',[
  ['diMAD','Corrente MAD (8 janelas)'],
  ['di','Corrente DFT 60 Hz'],
  ['dv','Tensao DFT 60 Hz'],
  ['dp','Potencia ativa fundamental DFT'],
  ['ds','Potencia aparente fundamental DFT'],
  ['dfp','FP fundamental (magnitude)'],
  ['dfluxo','Sentido DFT'],
  ['dei','Erro absoluto de corrente'],
  ['dep','Erro absoluto de potencia'],
  ['deip','Erro relativo de corrente'],
  ['depp','Erro relativo de potencia']
]);

cards('gridFiltro',[
  ['sbruto','RMS bruto'],
  ['sfiltrado','RMS filtrado'],
  ['sbase','Ruido-base'],
  ['sdispersao','Dispersao'],
  ['sentrada','Limiar de entrada'],
  ['ssaida','Limiar de saida'],
  ['sestado','Estado do filtro']
]);

// =====================================================
// WEBSOCKET - DADOS REAIS
// =====================================================

function atualizarReal(d) {

  set('rI60',d.ajusteOk?fmt(d.i60,3,'mV'):'--');
  set('rV60',d.ajusteOk?fmt(d.v60,3,'mV'):'--');
  set('rBI',fmt(d.baseI60,3,'mV'));set('rBV',fmt(d.baseV60,3,'mV'));
  set('rFS',fmt(d.fsReal,1,'Hz'));set('rLate',d.atrasos);
  set('rEI',d.extI);set('rEV',d.extV);set('rOK',d.ajusteOk?'OK':'Falha');
  ondasFisicas(d);
  set('estadoReal',
    d.calibrado
    ? 'CALIBRADO'
    : d.ruidoCalibrado
    ? 'BANCADA - Calibracao eletrica pendente'
    : 'Calibrando ruido...');

  set('tensao',fmt(d.tensao,1,'V'));
  set('corrente',fmt(d.corrente,3,'A'));
  set('potencia',fmt(d.potencia,1,'W'));
  set('aparente',fmt(d.aparente,1,'VA'));
  set('fp',fmt(d.fp,3));
  set('fluxo',d.fluxo);

  set('energiaImport',fmt(d.energiaImport,6,'kWh'));
  set('energiaExport',fmt(d.energiaExport,6,'kWh'));

  for(const id of [
    'offsetI','offsetV','rmsIbruto',
    'rmsIfiltrado','rmsVadc','ruidoBase',
    'dispersaoRuido','limiarEntrada','limiarSaida'
  ]) {
    set(id,fmt(d[id],id.startsWith('offset')?3:4,'V'));
  }

  set('acimaLimiar',
    !d.ruidoCalibrado
    ? 'CALIBRANDO'
    : d.acimaLimiar
    ? 'OSCILACAO ACIMA'
    : 'DENTRO DO RUIDO');
}

function conectar() {
  const socket =
    new WebSocket('ws://'+location.host+'/ws');

  socket.onopen=()=>{
    set('conexao','Conectado');
  };

  socket.onmessage=e=>{
    try{
      atualizarReal(JSON.parse(e.data));
    }catch(err){
      console.log(err);
    }
  };

  socket.onclose=()=>{
    set('conexao','Desconectado');
    setTimeout(conectar,2000);
  };
}

conectar();

// =====================================================
// CONTROLE DA SIMULACAO
// =====================================================

let executando=false;
let temporizador=null;
let energiaI=0;
let energiaE=0;
let segundos=0;
let ultimo=null;
let geracao=0;
let modoSim=false;

function mudarModo(sim){

  modoSim=sim;

  $('real').classList.toggle('hidden',sim);
  $('sim').classList.toggle('hidden',!sim);

  $('btnReal').classList.toggle('ativo',!sim);
  $('btnSim').classList.toggle('ativo',sim);

  clearTimeout(temporizador);
  geracao++;

  if(sim){
    atualizarSim(geracao,false);
  }else{
    executando=false;
    set('iniciar','Iniciar');
  }
}

function desenharOndas(d){

  const cv=$('ondas');
  const w=cv.clientWidth;
  const h=cv.clientHeight;

  if(!w||!h)return;

  const escala=window.devicePixelRatio||1;

  cv.width=w*escala;
  cv.height=h*escala;

  const ctx=cv.getContext('2d');
  ctx.scale(escala,escala);

  const l=45;
  const r=w-45;
  const meio=h/2;
  const amp=h/2-25;

  ctx.font='11px Arial';
  ctx.fillStyle='#60758a';
  ctx.strokeStyle='#d8e1eb';

  for(let k=0;k<=4;k++){

    let x=l+(r-l)*k/4;

    ctx.beginPath();
    ctx.moveTo(x,15);
    ctx.lineTo(x,h-23);
    ctx.stroke();

    ctx.textAlign='center';

    ctx.fillText(
      (k*1000/120).toFixed(1),
      x,h-5);
  }

  ctx.beginPath();
  ctx.moveTo(l,meio);
  ctx.lineTo(r,meio);
  ctx.stroke();

  ctx.textAlign='left';
  ctx.fillText('+180 V',0,20);
  ctx.fillText('0',20,meio);
  ctx.fillText('-180 V',0,h-25);

  ctx.textAlign='right';
  ctx.fillText('+7,1 A',w,20);
  ctx.fillText('0',w-10,meio);
  ctx.fillText('-7,1 A',w,h-25);
  ctx.fillText('ms',w,h-5);

  function onda(cor,a,fase){

    ctx.strokeStyle=cor;
    ctx.lineWidth=2;
    ctx.beginPath();

    for(let k=0;k<=240;k++){

      let x=l+(r-l)*k/240;

      let y=meio-amp*a*
        Math.sin(4*Math.PI*k/240-fase);

      if(k===0)ctx.moveTo(x,y);
      else ctx.lineTo(x,y);
    }

    ctx.stroke();
  }

  onda('#2476c5',Math.SQRT2*d.v/180,0);

  onda('#d78a18',
    Math.SQRT2*d.i/7.1,d.fase);
}

async function atualizarSim(g,integrar){

  if(!modoSim||g!==geracao)return;

  try{

    const caso=$('caso').value;
    const ruido=$('ruido').checked?1:0;

    const resposta=await fetch(
      '/api/simulacao?caso='+caso+'&ruido='+ruido,
      {cache:'no-store'});

    if(!resposta.ok)
      throw Error('Falha HTTP');

    const d=await resposta.json();

    if(g!==geracao||!modoSim)return;

    ultimo=d;

    if(executando&&integrar){
      energiaI+=d.eImport;
      energiaE+=d.eExport;
      segundos+=60;
    }

    set('sv',fmt(d.v,1,'V'));
    set('si',fmt(d.i,3,'A'));
    set('sp',fmt(d.p,1,'W'));
    set('ss',fmt(d.s,1,'VA'));

    set('sfp',d.fp===null?'--':fmt(d.fp,3));

    set('sfluxo',d.fluxo);

    set('sei',fmt(energiaI,6,'kWh'));
    set('see',fmt(energiaE,6,'kWh'));

    set('diMAD',fmt(d.iMAD,4,'A'));
    set('di',fmt(d.iDFT,4,'A'));
    set('dv',fmt(d.vDFT,2,'V'));
    set('dp',fmt(d.pDFT,2,'W'));
    set('ds',fmt(d.sDFT,2,'VA'));
    set('dfp',d.fpDFT===null?'--':fmt(d.fpDFT,4));
    set('dfluxo',d.fluxoDFT);
    set('dei',fmt(d.erroI,6,'A'));
    set('dep',fmt(d.erroP,6,'W'));
    set('deip',d.erroIpct===null?'Indefinido':fmt(d.erroIpct,6,'%'));
    set('depp',d.erroPpct===null?'Indefinido':fmt(d.erroPpct,6,'%'));
    set('sbruto',fmt(d.bruto*1000,2,'mV'));
    set('sfiltrado',fmt(d.filtrado*1000,2,'mV'));
    set('sbase',fmt(d.base*1000,2,'mV'));
    set('sdispersao',fmt(d.dispersao*1000,2,'mV'));
    set('sentrada',fmt(d.entrada*1000,2,'mV'));
    set('ssaida',fmt(d.saida*1000,2,'mV'));

    set('sestado',
      d.detectado
      ? 'ACIMA DO LIMIAR'
      : 'DENTRO DO RUIDO');

    set('tempoSim',
      'Tempo simulado: '+segundos+' s');

    desenharOndas(d);

  }catch(e){

    if(g===geracao)
      set('tempoSim','Falha de comunicacao com ESP32');
  }

  if(executando&&g===geracao&&modoSim){

    temporizador=setTimeout(
      ()=>atualizarSim(g,true),1000);
  }
}

function reiniciarConsulta(){

  clearTimeout(temporizador);
  geracao++;

  atualizarSim(geracao,false);
}

function alternarExecucao(){

  executando=!executando;

  set('iniciar',executando?'Pausar':'Iniciar');

  reiniciarConsulta();
}

function zerarEnergia(){

  executando=false;

  energiaI=0;
  energiaE=0;
  segundos=0;

  set('iniciar','Iniciar');

  reiniciarConsulta();
}

$('caso').onchange=reiniciarConsulta;
$('ruido').onchange=reiniciarConsulta;

window.addEventListener('resize',()=>{
  if(modoSim&&ultimo)
    desenharOndas(ultimo);
});

if(location.pathname==='/simulacao')
  mudarModo(true);

</script>
</body>
</html>
)rawliteral";

// =====================================================
// DISPLAY OLED
// =====================================================

void mostrarCalibracao() {
  oled.clearBuffer();
  oled.setFont(u8g2_font_6x10_tf);

  oled.drawStr(0, 12, "TCC - MEDIDOR");
  oled.drawHLine(0, 15, 128);

  oled.drawStr(0, 30, "Calibrando ruido I");

  char linha[30];

  snprintf(
      linha, sizeof(linha),
      "Janela: %u/%u",
      janelas, JANELAS_RUIDO);

  oled.drawStr(0, 46, linha);
  oled.drawStr(0, 62, "SEM REDE AC");

  oled.sendBuffer();
}

void mostrarOLED() {
  oled.clearBuffer();
  oled.setFont(u8g2_font_6x10_tf);

  char linha[32];

  if (!dadosCalibrados()) {

    oled.drawHLine(0, 13, 128);

    if ((millis() / 5000) % 2 == 0) {

      oled.drawStr(0, 10, "TCC - SEM REDE AC");

      snprintf(linha, sizeof(linha),
          "I off: %.3f V", leitura.offsetI);

      oled.drawStr(0, 25, linha);

      snprintf(linha, sizeof(linha),
          "I AC : %.4f V", leitura.filtradoI);

      oled.drawStr(0, 37, linha);

      snprintf(linha, sizeof(linha),
          "V off: %.3f V", leitura.offsetV);

      oled.drawStr(0, 49, linha);

      snprintf(linha, sizeof(linha),
          "V AC : %.4f V", leitura.acV);

      oled.drawStr(0, 61, linha);

    } else {

      oled.drawStr(0, 10, "TCC - FILTRO I");

      snprintf(linha, sizeof(linha),
          "Base: %.1f mV", ruidoBase * 1000);

      oled.drawStr(0, 25, linha);

      snprintf(linha, sizeof(linha),
          "Limiar: %.1f mV", limiarEntrada * 1000);

      oled.drawStr(0, 35, linha);

      oled.drawStr(0, 45,
          acimaLimiar
          ? "Sinal: acima limiar"
          : "Sinal: ruido");

      String ip = ipDoESP();

      snprintf(linha, sizeof(linha),
          "IP: %s", ip.c_str());

      oled.drawStr(0, 61, linha);
    }

  } else {

    oled.drawStr(0, 10, "TCC - BIDIRECIONAL");
    oled.drawHLine(0, 13, 128);

    snprintf(linha, sizeof(linha),
        "Tensao: %.1f V", leitura.vrms);

    oled.drawStr(0, 25, linha);

    snprintf(linha, sizeof(linha),
        "Corrente: %.2f A", leitura.irms);

    oled.drawStr(0, 37, linha);

    snprintf(linha, sizeof(linha),
        "P ativa: %.1f W", leitura.p);

    oled.drawStr(0, 49, linha);
    oled.drawStr(0, 61, leitura.fluxo);
  }

  oled.sendBuffer();
}

// =====================================================
// WIFI
// =====================================================

void iniciarWiFi() {

  if (strcmp(WIFI_SSID, "SUA_REDE") != 0) {

    WiFi.mode(WIFI_STA);
    WiFi.begin(WIFI_SSID, WIFI_SENHA);

    unsigned long inicio = millis();

    while (WiFi.status() != WL_CONNECTED &&
           millis() - inicio < 10000) {

      delay(250);
    }
  }

  if (WiFi.status() != WL_CONNECTED) {

    modoAP = true;
    WiFi.mode(WIFI_AP);

    WiFi.softAP(AP_SSID, AP_SENHA);

    Serial.println("Wi-Fi: TCC-Medidor");
  }

  Serial.print("Dashboard: http://");
  Serial.println(ipDoESP());
}

// =====================================================
// SETUP
// =====================================================

void setup() {

  Serial.begin(115200);

  delay(800);

  Wire.begin(SDA_OLED, SCL_OLED);

  oled.setI2CAddress(0x3C * 2);
  oled.begin();

  analogReadResolution(12);

  analogSetPinAttenuation(PINO_I, ADC_11db);
  analogSetPinAttenuation(PINO_V, ADC_11db);

  oled.clearBuffer();
  oled.setFont(u8g2_font_6x10_tf);

  oled.drawStr(0, 15, "TCC - MEDIDOR");
  oled.drawStr(0, 37, "Iniciando Wi-Fi...");

  oled.sendBuffer();

  iniciarWiFi();

  ws.onEvent([](
      AsyncWebSocket *srv,
      AsyncWebSocketClient *cliente,
      AwsEventType tipo,
      void *arg,
      uint8_t *dados,
      size_t len) {

    if (tipo == WS_EVT_CONNECT ||
        tipo == WS_EVT_DATA) {

      // O loop envia o proximo snapshot; evita leitura concorrente dos buffers.
    }
  });

  servidor.addHandler(&ws);

  servidor.on("/", HTTP_GET,
      [](AsyncWebServerRequest *pedido) {

    pedido->send_P(
        200, "text/html; charset=utf-8", PAGINA);
  });

  servidor.on("/simulacao", HTTP_GET,
      [](AsyncWebServerRequest *pedido) {

    pedido->send_P(
        200, "text/html; charset=utf-8", PAGINA);
  });

  servidor.on("/api/simulacao", HTTP_GET,
      [](AsyncWebServerRequest *pedido) {

    int caso = pedido->hasParam("caso")
        ? pedido->getParam("caso")->value().toInt()
        : 0;

    if (caso < 0 || caso > 5)
      caso = 0;

    bool ruido = !pedido->hasParam("ruido") ||
        pedido->getParam("ruido")->value() != "0";

    pedido->send(
        200,
        "application/json; charset=utf-8",
        calcularSimulacao(caso, ruido));
  });

  servidor.begin();

  Serial.println(
      "GPIO34 corrente | GPIO35 tensao");

  Serial.println(
      "Calibrando ruido - SEM REDE AC");

  instanteEnergia = millis();
}

// =====================================================
// LOOP PRINCIPAL - SENSORES REAIS
// =====================================================

void loop() {

  Estatisticas est;

  Ajuste60 ai,av;
  uint32_t inicio=micros(), primeiro=0, ultimo=0;
  atrasos=extI=extV=0;
  for(uint16_t i=0;i<AMOSTRAS;i++) {
    uint32_t alvo=inicio+((uint32_t)i*1000000UL)/1200;
    while((int32_t)(micros()-alvo)<0) delayMicroseconds(10);
    uint32_t agora=micros();
    if((int32_t)(agora-alvo)>833) atrasos++;
    analogRead(PINO_I); int rawI=analogRead(PINO_I);
    uint32_t t0=micros(); double sinalI=analogReadMilliVolts(PINO_I)/1000.0;
    uint32_t ti=t0+(micros()-t0)/2-inicio;
    analogRead(PINO_V); int rawV=analogRead(PINO_V);
    t0=micros(); double sinalV=analogReadMilliVolts(PINO_V)/1000.0;
    uint32_t tv=t0+(micros()-t0)/2-inicio;
    if(rawI<=10 || rawI>=4085) extI++;
    if(rawV<=10 || rawV>=4085) extV++;
    if(i==0) primeiro=ti; ultimo=ti;
    ai.add(sinalI,ti); av.add(sinalV,tv); est.adicionar(sinalI,sinalV);
    if(i<N_GRAF){grafI[i]=sinalI;grafV[i]=sinalV;grafTI[i]=ti;grafTV[i]=tv;}
  }
  fsReal=ultimo>primeiro?(AMOSTRAS-1)*1000000.0/(ultimo-primeiro):0;
  double di=0,ci=0,si=0,dv=0,cv=0,sv=0;
  bool oi=ai.solve(di,ci,si),ov=av.solve(dv,cv,sv);
  ajusteOk=oi&&ov;
  realI60=oi?hypot(ci,si)/sqrt(2.0):0;
  realV60=ov?hypot(cv,sv)/sqrt(2.0):0;
  Serial.printf("V4 60Hz: I %.3f mV | V %.3f mV | Fs %.1f Hz | atrasos %u | extremos I/V %u/%u\n",realI60*1000,realV60*1000,fsReal,atrasos,extI,extV);

  leitura.offsetI = est.mediaI;
  leitura.offsetV = est.mediaV;
  leitura.brutoI = est.rmsI();
  leitura.acV = est.rmsV();

  if (!ruidoCalibrado) {

    refI60[janelas]=realI60; refV60[janelas]=realV60;
    janelaRuido[janelas++] = leitura.brutoI;

    mostrarCalibracao();

    Serial.printf(
        "Calibracao ruido: %u/%u | RMS: %.4f V\n",
        janelas, JANELAS_RUIDO, leitura.brutoI);

    if (janelas >= JANELAS_RUIDO) {
      baseI60=mediana(refI60,JANELAS_RUIDO); baseV60=mediana(refV60,JANELAS_RUIDO);
      terminarCalibracaoRuido();
    }

    ws.textAll(criarJSON());
    ws.cleanupClients();

    return;
  }

  aplicarFiltro(leitura.brutoI);

  if (dadosCalibrados()) {

    leitura.vrms =
        leitura.acV * FATOR_V;

    leitura.irms =
        leitura.filtradoI * FATOR_I;

    leitura.s =
        leitura.vrms * leitura.irms;

    leitura.p =
        leitura.irms == 0
        ? 0
        : est.potenciaADC() * FATOR_V * FATOR_I;

    leitura.fp =
        leitura.s > 0
        ? fmin(1.0, fabs(leitura.p / leitura.s))
        : 0;

    if (leitura.p > 5)
      leitura.fluxo = "IMPORTANDO";

    else if (leitura.p < -5)
      leitura.fluxo = "EXPORTANDO";

    else
      leitura.fluxo = "SEM FLUXO";

    unsigned long agora = millis();

    double horas =
        (agora - instanteEnergia) / 3600000.0;

    if (leitura.p > 0) {

      leitura.energiaImport +=
          leitura.p * horas / 1000.0;

    } else {

      leitura.energiaExport +=
          -leitura.p * horas / 1000.0;
    }

    instanteEnergia = agora;

  } else {

    instanteEnergia = millis();

    leitura.fluxo = "CALIBRACAO PENDENTE";
  }

  mostrarOLED();

  Serial.println("-----------------------------");

  Serial.printf(
      "GPIO34 I: offset %.3f V | "
      "AC bruto %.4f V | AC filtrado %.4f V\n",
      leitura.offsetI,
      leitura.brutoI,
      leitura.filtradoI);

  Serial.printf(
      "GPIO35 V: offset %.3f V | AC %.4f V\n",
      leitura.offsetV,
      leitura.acV);

  Serial.printf(
      "I ruido: base %.4f | dispersao %.4f | "
      "limiar %.4f V | %s\n",
      ruidoBase,
      dispersaoRuido,
      limiarEntrada,
      acimaLimiar
      ? "OSCILACAO ACIMA"
      : "DENTRO DO RUIDO");

  Serial.println(
      dadosCalibrados()
      ? "Grandezas reais liberadas"
      : "V/I/P/Fluxo: aguardando calibracao");

  ws.textAll(criarJSON());

  ws.cleanupClients();

  delay(350);
}
