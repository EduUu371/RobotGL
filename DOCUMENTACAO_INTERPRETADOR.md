# Documentacao do interpretador de cinematica

O aplicativo usa um pequeno DSL inspirado em comandos industriais como `MoveJ`, `MoveL` e `MoveC`. O codigo e editado em uma janela separada da renderizacao 3D; quando o usuario clica em `Compilar`, cada ponto solicitado passa pela cinematica inversa do robo selecionado.

## Como executar

```powershell
pip install -r requirements.txt
python main.py
```

Para testar apenas a conversao de cinematica, abra `index.html` no navegador. Essa pagina nao possui calculos financeiros; ela converte ponto cartesiano para juntas e juntas para ponto cartesiano nos robos cartesiano e cilindrico.

## Robos disponiveis

- `ROBO cartesiano;`: tres juntas lineares, com movimento nos eixos X, Y e Z.
- `ROBO cilindrico;`: uma junta rotacional na base, uma junta linear radial e uma junta linear vertical.

Os limites ficam em `.robot_limits.json`, propositalmente separado do codigo do aluno/operador. Se um ponto passa do dominio, o interpretador gera erro semantico.

Na renderizacao, o dominio de trabalho aparece transparente:

- Robo cartesiano: paralelepipedo transparente com os limites de X, Y e Z.
- Robo cilindrico: cilindro oco transparente. O raio interno representa a area excluida por colisao/proximidade com o corpo do robo.

Os modelos foram desenhados para separar corpo e dominio:

- O cartesiano aparece como uma maquina tipo CNC/portal. Mesa, colunas e trilhos ficam fora ou acima do volume transparente; o volume representa a area alcançavel pela ponta da ferramenta.
- O cilindrico aparece como um poste central com braço radial. O poste fica no vazio interno do cilindro oco, portanto nao faz parte do dominio alcançavel do efetuador.

O menu superior permite trocar o robo e ligar/desligar extras:

- `Cartesiano` / `Cilindrico`: troca o modelo 3D e o modelo de cinematica usado pelo interpretador.
- `Dominio`: mostra ou esconde o volume de trabalho transparente.
- `Rastro`: mostra ou esconde o historico visual do efetuador.
- `GPU`: usa malhas mais detalhadas. Desligado, o simulador usa um fallback mais leve, com menos vertices, para computadores fracos.

## Edicao e compilacao

A caixa de texto do codigo nao e desenhada dentro da janela OpenGL. Ela fica em uma janela nativa separada chamada `Editor de codigo do robo`.

O fluxo esperado e:

- Editar o programa na caixa de texto.
- Clicar em `Compilar`.
- Conferir o status no editor e na barra superior da visualizacao 3D.
- A animacao 3D so muda depois de uma compilacao bem-sucedida.

## Cores das juntas

- Vermelho: junta rotacional.
- Azul ou verde: junta linear.
- Amarelo: estrutura/eixo auxiliar.

## Comandos

Todo comando termina com `;`.

```text
ROBO cartesiano;
PONTO_INICIAL(0, 0, 1);
VEL(1.0);
ZONA(0.05);
MoveL(2, 0, 1);
MoveJ(2, 1, 2, v=1.2);
MoveC(0, 0, 2, angulo=90, v=0.8);
```

### `PONTO_INICIAL(x, y, z);`

Define o ponto inicial do efetuador. Tambem aceita os aliases `INICIO` e `ORIGEM`.

### `VEL(valor);`

Define a velocidade padrao dos movimentos seguintes. O limite maximo depende do robo e fica no arquivo oculto de limites.

### `ZONA(valor);`

Define uma tolerancia/raio de aproximacao registrado junto do movimento. No simulador ela aparece como dado do comando e serve como base para discutir zonas industriais.

### `MoveL(x, y, z, v=opcional);`

Movimento linear no espaco cartesiano. O interpretador interpola varios pontos em linha reta, valida todos contra o dominio do robo e calcula a cinematica inversa de cada ponto.

### `MoveJ(x, y, z, v=opcional);`

Movimento por juntas. O destino e convertido por cinematica inversa, depois as juntas sao interpoladas. Em robos cilindricos isso mostra a diferenca entre movimentar as juntas e seguir uma reta perfeita no espaco.

### `MoveC(cx, cy, cz, angulo=graus, v=opcional);`

Movimento circular no plano XY. O ponto atual e usado como inicio do arco, `(cx, cy, cz)` e o centro e `angulo` define quanto o efetuador gira.

## Erros demonstrados

- Erro lexico: caractere nao permitido, por exemplo `@`.
- Erro sintatico: comando sem `;`, parenteses errado, numero invalido ou comando desconhecido.
- Erro semantico: ponto fora dos limites, velocidade acima do permitido, robo inexistente ou arco circular impossivel.

## Atalhos

- `F1`: carrega exemplo cartesiano.
- `F2`: carrega exemplo cilindrico.
- `F3`: carrega exemplo de teste da animacao.
- `F5`: recompila o ultimo codigo enviado pelo editor.
- Arrastar com mouse na area 3D: rotaciona a camera.
- Scroll do mouse: zoom.

## Exemplo de teste da animacao

O arquivo `exemplo_teste_animacao.robot` contem um programa pronto para colar no editor de codigo. Ele mistura `MoveJ`, `MoveL` e `MoveC` para verificar se o robo se move, se o rastro ciano e desenhado e se o dominio transparente aparece corretamente.
