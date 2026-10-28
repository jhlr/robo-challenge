#pragma once
// Borda do mapa: as ruas terminam numa linha VERMELHA (cima e baixo) ou AZUL (laterais), que atravessa a barra como um cruzamento (S1 e S8 no preto),
// mas nao e rua: nao ha o que girar para confirmar. O que diferencia e a COR da linha em que o robo esta (TCS34725 do chao). Pura: so conta.
// A tabela de cores (referencias do TCS) e dona do firmware principal (core/color_table.h); o sketch classifica e passa so CorLinha para ca.
// O beco depois do ultimo cruzamento e curto (~127 mm, quarteiroes menores na borda), entao a borda aparece logo depois de um cruzamento: so a cor decide.
#include <stdint.h>

namespace seguidor {

enum class CorLinha : uint8_t { Outra, Vermelho, Azul };   // Outra = preto, branco, verde... ou nao se sabe

// O que fazer quando as duas pontas acendem (candidato a cruzamento) e a cor do chao ja foi lida.
enum class AcaoCruzamento : uint8_t {
  Cruzamento,   // cor normal: confirma girando 90 graus como sempre
  MeiaVolta,    // vermelho ou azul sem cubo na mao: gira 180 e volta para dentro do mapa (para longe do verde / da borda)
  Navegador,    // vermelho ou azul COM cubo: nao da meia-volta sozinho, quem decide a rota e o navegador (leva o cubo ao verde)
};

// `temCubo` = esta carregando um cubo para guardar.
inline AcaoCruzamento acaoNaPonta(CorLinha cor, bool temCubo) {
  if (cor == CorLinha::Outra) return AcaoCruzamento::Cruzamento;
  return temCubo ? AcaoCruzamento::Navegador : AcaoCruzamento::MeiaVolta;
}

// Varias leituras de cor seguidas (o robo esta parado): so acredita se TODAS concordam; senao Outra (cai no giro de confirmacao, que
// reprova uma borda porque nao ha rua dos lados).
inline CorLinha corConfirmada(const CorLinha* leituras, int n) {
  if (n <= 0) return CorLinha::Outra;
  for (int i = 1; i < n; i++) {
    if (leituras[i] != leituras[0]) return CorLinha::Outra;
  }
  return leituras[0];
}

// O TCS pode estar a frente ou atras da barra: a cor da borda e vista numa hora e as pontas acendem em outra. Entao guarda a ultima cor
// NAO normal que viu e usa se foi ha pouco (janelaMs), em vez de depender de onde o TCS esta quando o robo para.
struct MemoriaCor {
  CorLinha cor = CorLinha::Outra;
  unsigned long visaEm = 0;   // +1 para nunca ser 0
};

inline void anotarCor(MemoriaCor& m, CorLinha lida, unsigned long agoraMs) {
  if (lida == CorLinha::Outra) return;
  m.cor = lida;
  m.visaEm = agoraMs + 1;
}

inline CorLinha corRecente(const MemoriaCor& m, unsigned long agoraMs, unsigned long janelaMs) {
  return m.visaEm != 0 && agoraMs + 1 - m.visaEm <= janelaMs ? m.cor : CorLinha::Outra;
}

// Depois de tratar a borda (meia-volta), esquece a cor: a linha vermelha que ficou atras nao conta para o proximo cruzamento.
inline void esquecerCor(MemoriaCor& m) { m = MemoriaCor{}; }

}  // namespace seguidor
