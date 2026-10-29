#pragma once
// Borda do mapa: as ruas terminam numa linha VERMELHA (cima e baixo) ou AZUL (laterais), que atravessa a barra como um cruzamento (S1 e S8 no preto),
// mas nao e rua: nao ha o que girar para confirmar. O que diferencia e a COR da linha em que o robo esta (TCS34725 do chao). Pura: so conta.
// A tabela de cores (referencias do TCS) e dona do firmware principal (core/color_table.h); o sketch classifica e passa so CorLinha para ca.
// O beco depois do ultimo cruzamento e curto (~127 mm, quarteiroes menores na borda), entao a borda aparece logo depois de um cruzamento: so a cor decide.
#include <stdint.h>

namespace seguidor {

enum class CorLinha : uint8_t { Outra, Vermelho, Azul, Verde };   // Outra = preto, branco... ou nao se sabe. Verde e o piso da zona livre (nao e linha)

// O que fazer quando as duas pontas acendem (candidato a cruzamento) e a cor do chao ja foi lida.
enum class AcaoCruzamento : uint8_t {
  Cruzamento,   // cor normal: confirma girando 90 graus como sempre
  MeiaVolta,    // vermelho ou azul sem cubo na mao: gira 180 e volta para dentro do mapa (para longe do verde / da borda)
  Navegador,    // vermelho ou azul COM cubo: nao da meia-volta sozinho, quem decide a rota e o navegador (leva o cubo ao verde)
  Portao,       // a PRIMEIRA azul depois de comecar (ou de ser reposicionado) e o nosso portao: segue em frente, nao e borda nem cruzamento
};

// `temCubo` = esta carregando um cubo para guardar. `portaoPendente` = ainda nao passou pelo nosso portao azul (verdadeiro ao ligar e depois de reposicionado):
// a primeira azul lida e a nossa (o robo comeca no verde, entre o amarelo e essa azul); a azul do outro lado e do adversario e vira borda.
// Levando um cubo para entregar, quem planeja liga `portaoPendente` ao sair pelo nosso portao: ai a azul tambem e Portao (com cubo na mao).
inline AcaoCruzamento acaoNaPonta(CorLinha cor, bool temCubo, bool portaoPendente = false) {
  if (cor == CorLinha::Outra || cor == CorLinha::Verde) return AcaoCruzamento::Cruzamento;   // verde e o piso da zona livre, nao e uma linha
  if (cor == CorLinha::Azul && portaoPendente) return AcaoCruzamento::Portao;   // esperava o nosso portao (entrando OU saindo para entregar um cubo): nao e borda
  return temCubo ? AcaoCruzamento::Navegador : AcaoCruzamento::MeiaVolta;
}

// Verde sob o TCS depois de ja ter passado pelo nosso portao (e sem estar levando um cubo para la): o robo foi pego e posto de volta no verde (reinicio, que o
// juiz faz). A pose volta para "no verde, de frente" e o mapa de cubos fica (os cubos nao se mexem sozinhos). `entregando` = foi de proposito deixar o cubo no verde.
inline bool reposicionadoPeloVerde(CorLinha cor, bool portaoPendente, bool entregando) { return cor == CorLinha::Verde && !portaoPendente && !entregando; }

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
