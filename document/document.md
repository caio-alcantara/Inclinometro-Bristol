# Inclinômetro Bristol

## 1. Introdução à Bristol e ao problema

&emsp;"A Bristol é uma empresa genuinamente brasileira, que se destaca na fabricação de implementos agrícolas ligados ao ramo da motosserra e na fabricação de perfuratrizes e brocas, amplamente utilizados na construção civil. Sua linha de produtos é bastante extensa, suprindo as necessidades de uma enorme gama de consumidores no Brasil e exterior." 

&emsp;Dessa forma, a Bristol se coloca como uma empresa B2B que vende, para empresas do setor agrícola, construção civíl, etc, máquinas que dão suporte nessas determinadas áreas. Para este projeto, foi indicado um problema/necessidade: a falta de uma maneira simples e rápida para se medir a inclinação de uma máquina perfuratriz. Medir a inclinação de uma perfuratriz é algo útil por diversos motivos, incluindo:
* Precisão no Posicionamento: Em muitos projetos de perfuração, como na indústria de petróleo, gás ou mineração, é de extrema importância que a perfuração siga um ângulo exato para atingir um ponto ou camada específica do subsolo. Medir a inclinação ajuda a garantir que o furo esteja na direção correta.
* Segurança Operacional: Uma perfuração desalinhada pode resultar em falhas no equipamento, vazamentos ou até acidentes. Medir a inclinação permite monitorar a trajetória da perfuratriz e ajustar o ângulo conforme necessário para evitar problemas.
* Eficiência e Custo: A perfuração em ângulos inadequados pode levar a um maior consumo de recursos, como tempo e energia. Controlar a inclinação ajuda a otimizar o processo, tornando-o mais eficiente e econômico.
* Controle da Direção: Em algumas operações, como a perfuração direcional, é necessário controlar a trajetória para chegar a alvos específicos. Medir a inclinação é essencial para garantir que o furo siga o caminho desejado.

&emsp;Com todos esse becefícios expostos, fica claro que não possuir uma maneira simples, rápida e direta de medir essa inclinação logo pode se tornar uma dor e um ponto de gargalo para a produção. Assim, o presente projeto tem como objetivo solucionar este projeto, servindo como um analgésico para esta dor. 

## 2. Apresentação da solução

&emsp;De início, foi definido um escopo que conseguísse suprir todas as necessidades da Bristol. A solução deveria suprir a necessidade de medir a inclinação do maquinário enquanto opera em situações adversas, incluindo altos níveis de vibração, alta temperatura e presença de poeira e/ou água. A partir desses requisitos, a solução pensada envolve um dispositivo IOT (Internet das Coisas) que será acoplado em uma perfuratriz. O dispositivo deverá utilizar sensores para medir a inclinação da perfuratriz em relação ao solo e enviar esse dado via BLuetooth para o celular do operador, que deve poder se conectar ao dispositivo com o seu celular e visualizar os dados de maneira simples. Além disso, todo o case/carenagem do dispositivo deve ser desenvolvido tendo em mente as condições de alta presença de poeira, temperaturas altas (50°C) e autonomia de bateria de pelo menos 24 horas. Mais detalhes sobre os requisitos funcionais e não-funcionais podem ser encontrados na seção X.X.

### 2.1. Divisão de fases do projeto

&emsp;A principio, foi acordo com a Bristol que o projeto seria desenvolvido ao longo de 5 meses a fim de se entregar uma versão completa e pronto para uso. Além disso, foram definidos entregáveis para cada mês de execução do projeto, levando em consideração que o primeiro mês é voltado para compra e teste de componentes, bem como desenvolvimento preliminar de uma placa inicial para validação da ideia. As entregas são como seguem:

**Entregáveis**:
- Protótipo funcional em protoboard com ESP32 e MPU9250.
- Firmware básico para:
  1. Leitura dos dados do inclinômetro (ângulos em X e Y).
  2. Cálculo do ângulo de inclinação em relação ao solo.
  3. Transmissão dos dados via Bluetooth para um aplicativo simples (ex.: interface básica ou software como Serial Bluetooth Terminal).
- Projeto e fabricação de uma PCI inicial simples para substituir a protoboard, com:
     - Conexões para ESP32, acelerômetro e bateria.
- Teste do dispositivo em campo para verificar funcionamento básico.
- Documentação inicial sobre os componentes usados e instruções para testes.
- Case protótipo: Desenvolvimento inicial de um protótipo de case simples, a ser desenvolvido em máquina corte a laser ou impressora 3D (ainda não resistente a água/poeira).

#### 2º Mês - Protótipo Aprimorado
**Objetivo**: Aprimorar o protótipo básico, implementar funcionalidades extras e preparar o dispositivo para um ambiente mais robusto.

**Entregáveis**:
- Firmware aprimorado com:
  1. Indicação do nível da bateria.
  2. Armazenamento interno dos dados de inclinação (em caso de perda de conexão Bluetooth).
  3. Otimização do cálculo de inclinação.
- Melhorias na PCI: Projeto de uma segunda versão com:
  1. Layout otimizado para resistência a vibração.
  2. Melhor proteção contra ruídos e falhas.
  3. Inclusão de LED para status de ligação e conexão Bluetooth.
- Implementação de uma interface web simples para exibir dados de inclinação e nível da bateria (em desenvolvimento inicial).

#### 3º Mês - Protótipo Avançado
**Objetivo**: Desenvolver um protótipo mais próximo da versão final, com foco em funcionalidades de software e conectividade.

**Entregáveis**:
- Firmware avançado com:
  1. Atualização de firmware via web (OTA).
  2. Melhorias na estabilidade da conexão Bluetooth.
  3. Modos de operação otimizados para economia de energia.
- Interface web funcional:
  1. Dashboard simples para exibir inclinação em tempo real, histórico e nível da bateria. Incluindo uma versão a ser utilizada como app mobile.

#### 4º Mês - Versão Pré-Final
**Objetivo**: Refinar o hardware e software, incluindo integração de todas as funcionalidades. Aprimorar case.

**Entregáveis**:
- Placa de circuito avançada: Projeto finalizado para uma versão industrial robusta, incluindo a utilização de CI's ao invés de módulos pré-prontos.
- Testes de resistência à vibração e temperatura.
- Firmware quase final:
  1. Últimas correções e ajustes de funcionalidades.
  2. Sincronização confiável com a interface web.
  3. Interface web refinada: Melhorias visuais e de usabilidade.
  4. Gráficos de histórico.
  5. Indicadores de status do dispositivo (ex.: conectado/desconectado).

#### 5º Mês - Versão Final e Entrega
**Objetivo**: Finalizar o dispositivo e preparar para implantação.

**Entregáveis**:
- Protótipo finalizado:
  1. Placa de circuito impressa final (projeto industrial).
  2. Case resistente a água, poeira e vibração.
  3. Firmware completamente funcional e atualizado.
  4. Interface web completa:
     1. Visualização em tempo real e histórico de dados.
     2. Funcionalidade para atualização remota do firmware.
  5. Documentação técnica completa:
     1. Esquemas do hardware.
     2. Código-fonte comentado.
     3. Guia de uso e manutenção.
     4. Testes finais de validação do produto.
  6. Apresentação do protótipo final ao cliente.

## 3. Persona 

&emsp;Personas são descrições fictícias, porém precisas, de possíveis usuários da solução a ser desenvolvida. A análise e criação de personas é um passo fundamental no desenvolvimento de um projeto, independente de seu escopo. A mesma tem a função de guiar o processo de criação, tendo sempre um foco primordial na pessoa que utilizará e manipulará sua solução (Marúas, 1997). Se bem feita, torna o nível de acurácia e satisfação do produto final maior, mais abrangente e satisfatório.

<div align="center">

<sub>Figura 1 - Persona João Silva parte 1 </sub>

   <img src="../assets/persona_1.png">

<sup>Fonte: Material produzido pelos autores (2024)</sup>

</div>

&emsp;João Silva é a principal persona deste projeto. Ele representa justamente o principal usuário do dispositivo: o operador de máquinas pesadas como a perfuratriz. João é um operador natual de São Paulo, com um ensino técnico em operação de maquinário pesado. No seu dia-a-dia, ele opera a perfuratriz por longas horas, realizando perfurações precisas para fundações de edifícios e projetos de infraestrutura. Além de operar a máquina, João é responsável por garantir que o equipamento esteja em boas condições e que as metas diárias de perfuração sejam atingidas.

<div align="center">

<sub>Figura 2 - Persona João Silva parte 2 </sub>

   <img src="../assets/persona_2.png">

<sup>Fonte: Material produzido pelos autores (2024)</sup>

</div>

&emsp;João, infelizmente, é Corinthiano... Entretanto, isso não o impede de valorizar extremamente a segurança em seu trabalho. Entretanto, ele sempre gosta de deixar clara a sua intenção de manter a eficiência mesmo com constantes verificações de segurança. Em relação ao uso de tecnologias para facilitar a sua vida, João prefere soluções diretas, que não precisem de muitas etapas para serem utilizadas. Em seu trabalho, a sua maior dor é a falta de um sistema de monitoramento de inclinação deixa a máquina sujeita a erros ou desgastes desnecessários. Sua maior necessidade envolve, dessa forma, um equipamento capaz de medir a inclinação da perfuratriz e que suporte as condições adversas da obra, como poeira, vibrações e altas temperaturas.

&emsp;A criação de João como persona para o projeto é essencial para que seja possível entender as necessidades, desafios e motivações de um público específico que o projeto pretende atingir. Ele possui as características de um trabalhador que é acostumado a operar máquinas pesadas e permite que a solução seja desenvolvida de maneira estratégica para sanar as suas dores. Assim, não é exagero algum dizer que João molda completamente o projeto, definindo muito de como o trabalho será feito.  


## X. Referências
MARÍAS, Julián. Persona. Alianza, 1997. Disponível em: http://www.hottopos.com/mp2/mariaspers.htm. Acesso em: 22 jan. 2025.


