# VSS
Este repositório servirá para manter os códigos que serão utilizados em minha Iniciação Científica de VSSL, feita com o professor [Eduardo Todt](http://www.inf.ufpr.br/todt/).
## Objetivo:
O objetivo inicial é de desenvolver um novo firmware baseado no anteriormente desenvolvido para o robô da categoria de [IEEE Very Small Size Soccer](http://www.cbrobotica.org/?page_id=81).

## Bibliotecas adicionais:
### Arduino:
Tirando a biblioteca Motorino.h, todas as demais podem ser obtidas por meio do próprio menu do software do Arduino  (Sketch -> Incluir Biblioteca -> Gerenciar Bibliotecas).
A instalação do Motorino.h e demais informações estão demonstradas no trabalho feito pelo Alexandre.

#### Breve explicação destas bibliotecas:
- [RF24.h](https://github.com/maniacbug/RF24) e [RF24Network.h](http://maniacbug.github.io/RF24Network/index.html) servem para que a comunicação sem-fio aconteça entre as placas
- [SPI.h](https://www.arduino.cc/en/reference/SPI) serve para o uso de protocolos de microcontroladores síncronos

### Python:
Todas as bibliotecas são instaladas com o uso do pip, que normalmente já vem instalado junto com o Python. Mas caso você não tenha: [veja este guia](https://pip.pypa.io/en/stable/installing/).

- [getch](https://pypi.org/project/getch/):
Basimente usado para a função de para pegar o input do usuário sem a necessidade de teclar o enter.
Pode ser instalado com o seguinte comando:
```
python -m pip install getch
```

- [serial](https://pythonhosted.org/pyserial/)
Utilizado para que ocorra a conexão via serial entre o programa Python e (no caso) a base tranmissora de dados.
```
python -m pip install pyserial
```
**Cuidado:** não instale a biblioteca 'serial' no lugar da 'pyserial'. O programa **não fuicionará** com esta outra biblioteca.
