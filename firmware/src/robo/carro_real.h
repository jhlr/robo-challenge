// O carro de verdade para a logica de seguidor/robo.h: a mesma interface do Carro simulado (veja o cabecalho de robo.h), sobre hal/ e a barra RC.
// Tudo o que e hardware fica em hal/ (rodas STS, sonar, TCS34725); aqui so se traduz: PWM do firmware -> velocidade das rodas, cor do TCS -> cor de linha/cubo/piso.
#pragma once
#include <Arduino.h>
#include <seguidor/borda.h>
#include <seguidor/inercial.h>
#include <seguidor/robo.h>
#include <seguidor/sensores_rc.h>

#include "core/color_table.h"
#include "hal/drive.h"
#include "hal/sensors.h"
#include "params.h"
#include "pins.h"

namespace robo {

// Numeros ainda NAO medidos neste carro (confira na pista): onde fica o sonar em relacao ao eixo nao importa aqui (o sonar le a propria distancia), mas a janela em que o TCS da
// frente enxerga um cubo ou uma plataforma sim.
constexpr float kAlcanceCorFrenteMm = 110.0f;   // o TCS da frente so vale quando o sonar ve algo mais perto que isto (senao ele leria o chao ou a parede ao fundo)
constexpr uint32_t kSonarTimeoutUs = 8000;      // ~1,4 m: a varredura de plataformas olha ate ~900 mm; sem eco nao trava o ciclo por 25 ms
constexpr unsigned long kPeriodoTcsMs = 60;     // a leitura do TCS leva 24 ms: nao ler a cada ciclo

class CarroReal {
 public:
  CarroReal(hal::StsDrive& drive, seguidor::BarraRC& barra, hal::Ultrasonic& sonar, hal::ColorSensor& chao, hal::ColorSensor& frente, float dtMs)
      : drive_(drive), barra_(barra), sonar_(sonar), chao_(chao), frente_(frente), dtMs_(dtMs) {}

  // ---- tempo e ciclo: espera o proximo ciclo de dtMs, le a barra e as rodas
  unsigned long agoraMs() const { return millis(); }
  void avancar() {
    while (static_cast<float>(micros() - ultimoCicloUs_) < dtMs_ * 1000.0f) {}
    ultimoCicloUs_ = micros();
    seguidor::ler(barra_);
    odoE_ = drive_.leftMm();
    odoD_ = drive_.rightMm();
  }

  // ---- atuadores
  void comandar(int esq, int dir) {
    alvoE_ = esq;
    alvoD_ = dir;
    drive_.setSpeeds(fracao(esq), fracao(dir));
  }
  void parar() { comandar(0, 0); }
  void compensar(double, double) {}   // o servo regula a velocidade de cada roda: nada a compensar
  double roda(bool esquerda) const { return esquerda ? alvoE_ : alvoD_; }
  // O braco ainda nao esta calibrado (angulos da garra e do ombro por medir): nao se mexe sem a validacao do usuario. A logica segue como se tivesse pegado.
  void coletar(int) { evento("COLETAR: braco/garra nao acionados nesta versao (falta calibrar); segue como se tivesse pegado"); }

  // ---- sensores
  void ler(bool preto[seguidor::kSensores]) const {
    for (int i = 0; i < seguidor::kSensores; i++) preto[i] = barra_.preto[i];
  }
  double odoEMm() const { return odoE_; }
  double odoDMm() const { return odoD_; }
  double odoMm() const { return (odoE_ + odoD_) / 2.0; }
  double us() {   // mm ate o que esta a frente, ou -1
    const uint16_t mm = sonar_.readMm(kSonarTimeoutUs);
    return mm == 0xFFFF ? -1.0 : mm;
  }
  seguidor::Piso piso() {
    switch (corDoChao()) {
      case core::VERMELHO: return seguidor::kPisoVermelho;
      case core::AZUL: return seguidor::kPisoAzul;
      case core::VERDE: return seguidor::kPisoVerde;
      case core::AMARELO: return seguidor::kPisoAmarelo;
      case core::PRETO: return seguidor::kPisoPreto;
      default: return seguidor::kPisoBranco;
    }
  }
  seguidor::CorLinha tcsChao() {
    switch (corDoChao(true)) {
      case core::VERMELHO: return seguidor::CorLinha::Vermelho;
      case core::AZUL: return seguidor::CorLinha::Azul;
      case core::VERDE: return seguidor::CorLinha::Verde;
      default: return seguidor::CorLinha::Outra;
    }
  }
  // O TCS da frente so conta quando ha algo perto (sonar): cor do cubo que esta a frente.
  seguidor::CorCubo tcsFrente() { return algoPerto(kAlcanceCorFrenteMm) ? corCubo(corDaFrente()) : seguidor::CorCubo::Nenhuma; }
  seguidor::CorCubo plataformaNaFrente(double alcanceMm) { return algoPerto(static_cast<float>(alcanceMm)) ? corCubo(corDaFrente()) : seguidor::CorCubo::Nenhuma; }
  int cuboNaFrente(double alcanceMm) { return algoPerto(static_cast<float>(alcanceMm)) ? 0 : -1; }   // so ha UM "cubo" para pegar: o que o sonar ve
  seguidor::Imu imu() const { return {0.0f, 0.0f, 0.0f, 1.0f}; }   // sem MPU: neutro (a logica nao usa o acelerometro com rumoPeloEncoder)

  // ---- missao
  bool missao() const { return true; }
  bool usAtivo() const { return true; }
  unsigned long tcsPeriodoMs() const { return kPeriodoTcsMs; }
  int consumirToques() { return 0; }   // o juiz sinaliza; o carro nao sabe
  void mostrarCarga(int, int) {}
  void limparCarga() {}
  void evento(const std::string& texto) { Serial.printf("[%lu] %s\n", millis(), texto.c_str()); }
  seguidor::Estado estado() const { return {0.0, 0.0, 0.0}; }   // so o simulador sabe a verdade
  const std::string& ecoTipo() const { return ecoTipo_; }   // so depuracao


 private:
  // PWM do firmware (-255..255) -> fracao do comando das rodas. O simulador converte PWM em mm/s (2,31 mm/s por PWM: velReto 110 = 254 mm/s), mas o STS3215 so entrega ~kStsMaxSteps
  // passos/s (~150 mm/s com roda de 66 mm): a escala linear abaixo mantem a ordem das velocidades (giro < reta) em vez de saturar tudo acima de ~65 PWM. Ajuste kPwmPlenoSts na pista.
  static constexpr float kPwmPlenoSts = 160.0f;   // PWM do firmware que vira 100% do comando (NAO medido)
  static float fracao(int pwm) { return constrain(pwm / kPwmPlenoSts, -1.0f, 1.0f); }
  core::Cor corDoChao(bool forcar = false) {
    if (forcar || millis() - ultimaChaoMs_ >= kPeriodoTcsMs / 2) {
      ultimaChaoMs_ = millis();
      hal::ColorSample a;
      if (chao_.read(a)) {
        const core::Cor bruta = core::classificarCor(a.r, a.g, a.b, a.c);
        chaoAceita_ = core::filtrarCor(bruta, chaoAnterior_, chaoAceita_);   // so acredita se repetir em duas leituras
      }
    }
    return chaoAceita_;
  }
  core::Cor corDaFrente() {
    hal::ColorSample a;
    return frente_.read(a) ? core::classificarCor(a.r, a.g, a.b, a.c) : core::DESCONHECIDA;
  }
  bool algoPerto(float alcanceMm) {
    const double d = us();
    return d > 0.0 && d <= alcanceMm;
  }
  static seguidor::CorCubo corCubo(core::Cor c) {
    switch (c) {
      case core::BRANCO: return seguidor::CorCubo::Branco;
      case core::PRETO: return seguidor::CorCubo::Preto;
      case core::VERDE: return seguidor::CorCubo::Verde;
      case core::AMARELO: return seguidor::CorCubo::Amarelo;
      case core::AZUL: return seguidor::CorCubo::Azul;
      case core::VERMELHO: return seguidor::CorCubo::Vermelho;
      default: return seguidor::CorCubo::Nenhuma;
    }
  }

  hal::StsDrive& drive_;
  seguidor::BarraRC& barra_;
  hal::Ultrasonic& sonar_;
  hal::ColorSensor& chao_;
  hal::ColorSensor& frente_;
  float dtMs_;
  uint32_t ultimoCicloUs_ = 0;
  double odoE_ = 0.0, odoD_ = 0.0;
  int alvoE_ = 0, alvoD_ = 0;
  unsigned long ultimaChaoMs_ = 0;
  std::string ecoTipo_;
  core::Cor chaoAceita_ = core::DESCONHECIDA, chaoAnterior_ = core::DESCONHECIDA;
};

}  // namespace robo
