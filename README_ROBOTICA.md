# Aplicativo de cinematica robotica

Projeto em Python com renderizacao OpenGL via `pyglet`.

```powershell
pip install -r requirements.txt
python main.py
```

No Windows, tambem pode executar `iniciar_app.bat`, que verifica o Python, instala as dependencias e abre o simulador.

O projeto tambem inclui `index.html`, uma pagina estatica para converter cinematica direta e inversa sem abrir o simulador. Ela usa os mesmos modelos matematicos e limites principais:

- Cartesiano: `J1=x`, `J2=y`, `J3=z`.
- Cilindrico inverso: `theta=atan2(y,x)`, `raio=sqrt(x^2+y^2)`, `J3=z`.
- Cilindrico direto: `x=raio*cos(theta)`, `y=raio*sin(theta)`, `z=J3`.

A janela OpenGL mostra apenas a animacao 3D do robo. O codigo e editado em uma janela separada, com uma caixa de texto nativa e o botao `Compilar`; a visualizacao 3D so atualiza depois desse clique. O arquivo `.robot_limits.json` guarda os dominios de movimento usados nos calculos.

No menu superior voce pode trocar entre robo cartesiano e cilindrico, alem de habilitar/desabilitar o dominio transparente, o rastro do efetuador e o modo GPU. Se o computador estiver lento, desligue `GPU`, `Dominio` ou `Rastro` para usar uma renderizacao mais leve.

Visualmente, o cartesiano foi modelado como uma CNC/portal e o cilindrico como um poste com braço radial. O dominio transparente representa apenas a regiao alcançavel pelo efetuador, nao o volume ocupado pelo corpo do robo.

Arquivos principais:

- `main.py`: janela OpenGL, editor separado, botao de compilacao e animacao.
- `robot_language.py`: interpretador, erros e geracao da trajetoria.
- `kinematics.py`: cinematica direta/inversa e interpolacoes.
- `index.html`: conversor de cinematica direta/inversa para testes rapidos.
- `.robot_limits.json`: limites dos robos cartesiano e cilindrico.
- `DOCUMENTACAO_INTERPRETADOR.md`: explicacao para entregar junto do trabalho.
- `exemplo_teste_animacao.robot`: codigo de teste para colar no interpretador.
