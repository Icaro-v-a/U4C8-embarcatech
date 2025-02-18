# Atividade: Controle de servomotor por PWM.

Este projeto implementa um sistema com a ferramenta Pico SDK para simular o controle do ângulo de um servomotor

## Descrição do Projeto

Nesta prática, será necessário simular os seguintes componentes:

1) Microcontrolador Raspberry Pi Pico W.
2) Servomotor – motor micro servo padrão – Wokwi

### Requisitos do Projeto

1) Considerando a GPIO 22, defina a sua frequência de PWM para, aproximadamente, 50Hz – período de 20ms. (20% da nota)
2) Defina o ciclo ativo do módulo PWM para 2.400µs (microssegundos) – Ciclo de Trabalho (Duty Cycle) de 0,12%. isto ajustará a flange (braço) do servomotor para a posição de, 
aproximadamente, 180 graus. Aguarde 05 segundos nesta posição. (10% da nota)
3) Defina o ciclo ativo do módulo PWM para 1.470µs (microssegundos) – Ciclo de Trabalho (Duty Cycle) de 0,0735%. Isto ajustará a flange do servomotor para a posição de, aproximadamente, 90 graus. Aguarde 05 segundos nesta posição. (10% da nota)
4) Defina o ciclo ativo do módulo PWM para 500µs (microssegundos) – Ciclo de Trabalho (Duty Cycle) de 0,025%. Isto ajustará a flange do servomotor para a posição de, 
aproximadamente, 0 graus. Aguarde 05 segundos nesta posição. (10% da nota)
5) Após a realização das proposições anteriores, crie uma rotina para movimentação periódica do braço do servomotor entre os ângulos de 0 e 180 graus. Obs.: a movimentação da flange deve 
ser suave, recomenda-se o incremento de ciclo ativo de ±5µs, com um atraso de ajuste de 10ms. (35% da nota)
6) Com o emprego da Ferramenta Educacional BitDogLab, faça um experimento com o código deste exercício utilizando o LED RGB – GPIO 12. O que o discente observou no comportamento da iluminação do referido LED? (15% da nota)

## Tecnologias Utilizadas
- **Microcontrolador:** Raspberry Pi Pico W
- **Ambiente de Desenvolvimento:** VS Code
- **Simulador:** Wokwi
- **Linguagem:** C (com Pico SDK)

## Autor: Ícaro Vasconcelos Alvim

## Como Executar
1. Clone o repositório:
   ```bash
   git clone https://github.com/Icaro-v-a/U4C7-atividade1.git

2. Navegue até o diretório do projeto:
    ```bash
        cd U4C7-atividade1

3. Compile e execute o projeto utilizando o Pico SDK no VS Code.

## Link do Vídeo
O vídeo do experimento com o emprego da Ferramenta Educacional BitDogLab pode ser acessado [aqui](https://www.youtube.com/shorts/16MM5i9jo0k).
