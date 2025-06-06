package frc.robot.util;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.inputs.LoggableInputs;
import org.littletonrobotics.junction.networktables.LoggedNetworkInput;

/**
 * Gerencia múltiplos chooser enviados à SmartDashboard para a seleção sequencial de comandos (ou
 * outras opções). Cada slot da sequência é representado por um SendableChooser que compartilha as
 * mesmas opções.
 *
 * <p>Exemplo de utilização:
 *
 * <pre>
 *   // Cria um chooser sequencial com 10 slots
 *   private final LoggedSequentialDashboardChooser&lt;Supplier&lt;Command&gt;&gt; sequentialChooser =
 *       new LoggedSequentialDashboardChooser<>("SequentialChooser", 10);
 *
 *   // Adiciona opções (por exemplo, comandos para uma auto)
 *   sequentialChooser.addOption("Center-I", () -> new PathPlannerAuto("Center-I"));
 *   sequentialChooser.addOption("Left-I", () -> new PathPlannerAuto("Left-I"));
 *   sequentialChooser.addOption("Right-I", () -> new PathPlannerAuto("Right-I"));
 *
 *   // Em algum ponto, obtém a sequência escolhida:
 *   List&lt;Supplier&lt;Command&gt;&gt; commandSequence = sequentialChooser.get();
 *   // commandSequence contém 10 elementos, um para cada slot.
 * </pre>
 *
 * @param <V> Tipo da opção (por exemplo, Supplier&lt;Command&gt;)
 */
public class LoggedSequentialDashboardChooser<V> extends LoggedNetworkInput {
  private final String keyPrefix;
  private final int numSlots;
  // Array de choosers, um para cada slot da sequência.
  private final SendableChooser<String>[] choosers;
  // Array com os valores selecionados (as chaves das opções) em cada slot.
  private final String[] selectedValues;
  // Mapeia as chaves (strings) para o objeto do tipo V.
  private final Map<String, V> options = new HashMap<>();

  // LoggableInputs para registrar (e reproduzir) as escolhas de cada slot.
  private final LoggableInputs inputs =
      new LoggableInputs() {
        @Override
        public void toLog(LogTable table) {
          for (int i = 0; i < numSlots; i++) {
            table.put(keyPrefix + "_" + i, selectedValues[i]);
          }
        }

        @Override
        public void fromLog(LogTable table) {
          for (int i = 0; i < numSlots; i++) {
            selectedValues[i] = table.get(keyPrefix + "_" + i, selectedValues[i]);
          }
        }
      };

  /**
   * Cria um chooser sequencial com o prefixo da chave e a quantidade de slots desejada.
   *
   * @param keyPrefix Prefixo para as chaves dos choosers (cada slot usará keyPrefix_0, keyPrefix_1,
   *     etc.).
   * @param numSlots Número de slots (por exemplo, 10 para 10 comandos sequenciais).
   */
  public LoggedSequentialDashboardChooser(String keyPrefix, int numSlots) {
    this.keyPrefix = keyPrefix;
    this.numSlots = numSlots;
    // Cria o array de choosers – nota: devido a limitações do Java com arrays genéricos, usamos
    // cast.
    @SuppressWarnings("unchecked")
    SendableChooser<String>[] tempChoosers =
        (SendableChooser<String>[]) new SendableChooser[numSlots];
    choosers = tempChoosers;
    selectedValues = new String[numSlots];

    // Inicializa cada chooser e publica na SmartDashboard com uma chave única
    for (int i = 0; i < numSlots; i++) {
      choosers[i] = new SendableChooser<>();
      SmartDashboard.putData(keyPrefix + "_" + i, choosers[i]);
    }
    periodic();
    Logger.registerDashboardInput(this);
  }

  /**
   * Adiciona uma nova opção (comum a todos os slots) ao chooser.
   *
   * @param optionKey Nome da opção (usado também como chave para identificação).
   * @param value Valor associado à opção.
   */
  public void addOption(String optionKey, V value) {
    for (int i = 0; i < numSlots; i++) {
      choosers[i].addOption(optionKey, optionKey);
    }
    options.put(optionKey, value);
  }

  /**
   * Adiciona uma nova opção (comum a todos os slots) e define-a como padrão.
   *
   * @param optionKey Nome da opção (usado também como chave para identificação).
   * @param value Valor associado à opção.
   */
  public void addDefaultOption(String optionKey, V value) {
    for (int i = 0; i < numSlots; i++) {
      choosers[i].setDefaultOption(optionKey, optionKey);
    }
    options.put(optionKey, value);
  }

  /**
   * Retorna uma lista com as opções selecionadas em cada slot. Se em algum slot não houver seleção
   * (ou se o default não estiver definido), o valor correspondente poderá ser {@code null}.
   *
   * @return Lista de objetos do tipo V selecionados para cada slot.
   */
  public List<V> get() {
    List<V> result = new ArrayList<>();
    for (int i = 0; i < numSlots; i++) {
      result.add(options.get(selectedValues[i]));
    }
    return result;
  }

  /**
   * Retorna o array interno de SendableChooser, para uso na configuração do layout da dashboard.
   * Não utilize para ler os dados diretamente.
   *
   * @return Array de SendableChooser.
   */
  public SendableChooser<String>[] getSendableChoosers() {
    return choosers;
  }

  /**
   * Método periódico que atualiza as escolhas de cada slot e processa o log. Deve ser chamado
   * periodicamente (por exemplo, no loop principal do robô).
   */
  public void periodic() {
    if (!Logger.hasReplaySource()) {
      for (int i = 0; i < numSlots; i++) {
        selectedValues[i] = choosers[i].getSelected();
      }
    }
    Logger.processInputs(keyPrefix, inputs);
  }
}
