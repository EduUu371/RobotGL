// Exemplo para testar se a animacao, a trajetoria e a cinematica estao funcionando.
// Cole este codigo no painel "Codigo" do aplicativo.

ROBO cilindrico;
PONTO_INICIAL(1.2, 0.0, 0.7);
VEL(0.9);
ZONA(0.05);

MoveJ(2.5, 0.0, 1.0, v=1.0);
MoveL(2.5, 1.2, 1.5, v=1.2);
MoveC(0.0, 0.0, 1.5, angulo=110, v=0.7);
MoveL(-1.0, 2.2, 2.4, v=0.9);
MoveJ(1.4, -1.4, 1.2, v=1.1);
MoveC(0.0, 0.0, 1.2, angulo=-95, v=0.8);
