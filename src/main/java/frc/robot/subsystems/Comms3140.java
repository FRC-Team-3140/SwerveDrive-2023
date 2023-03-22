package frc.robot.subsystems;

import java.util.ArrayList;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/*
 * Manage persistant network table settings.
 */
public class Comms3140 extends SubsystemBase {

    static final double kEpsilon = 1e-5; // Threshold for double equals comparisons

    interface DoubleGetter {
        double run();
    }

    interface DoubleSetter {
        void run(double value);
    }

    interface IntGetter {
        int run();
    }

    interface IntSetter {
        void run(int value);
    }

    interface BooleanGetter {
        boolean run();
    }

    interface BooleanSetter {
        void run(boolean value);
    }

    interface StringGetter {
        String run();
    }

    interface StringSetter {
        void run(String value);
    }

    private abstract class SettingUpdater {
        double m_last_update = 0.0;
        String m_subsystem = null;
        String m_entry = null;
        TimePeriod m_period = TimePeriod.kEveryOneSeconds;

        void update(double current_time) {
            checkValue();
            m_last_update = current_time;
        }

        /**
         * Overide this method for each type.
         */
        abstract void checkValue();
    }

    private class DoubleUpdater extends SettingUpdater {

        final DoubleGetter m_getter;
        final DoubleSetter m_setter;
        double m_current_value = 0.0;
        final NetworkTableEntry m_table_entry;

        DoubleUpdater(String subsystem, String entry, DoubleGetter getter, DoubleSetter setter, double default_value,
                TimePeriod period) {
            this.m_subsystem = subsystem;
            this.m_entry = entry;
            this.m_getter = getter;
            this.m_setter = setter;
            this.m_period = period;

            m_table_entry = m_settings_table.getSubTable(subsystem).getEntry(entry);
            m_current_value = m_table_entry.getDouble(default_value);
            if (Math.abs(m_current_value - default_value) < kEpsilon) {
                System.out.printf("Warning: Settings3140/%s/%s entry %.5f does not equal default value of %.5f.\n",
                        subsystem, entry, m_current_value, default_value);
            }
            m_table_entry.setDouble(m_current_value);
            m_table_entry.setPersistent();
            setter.run(m_current_value);
        }

        @Override
        void checkValue() {
            double value = 0.0;

            // check the program value first
            value = m_getter.run();
            if (Math.abs(value - m_current_value) > kEpsilon) {
                m_table_entry.setDouble(value);
                m_current_value = value;
            }

            // check the networktable second
            value = m_table_entry.getDouble(m_current_value);
            if (Math.abs(value - m_current_value) > kEpsilon) {
                m_setter.run(value);
                m_current_value = value;
            }
        }
    }

    private class IntUpdater extends SettingUpdater {

        final IntGetter m_getter;
        final IntSetter m_setter;
        int m_current_value = 0;
        final NetworkTableEntry m_table_entry;

        IntUpdater(String subsystem, String entry, IntGetter getter, IntSetter setter, int default_value,
                TimePeriod period) {
            this.m_subsystem = subsystem;
            this.m_entry = entry;
            this.m_getter = getter;
            this.m_setter = setter;
            this.m_period = period;

            m_table_entry = m_settings_table.getSubTable(subsystem).getEntry(entry);
            m_current_value = (int) m_table_entry.getInteger(default_value);
            if (m_current_value != default_value) {
                System.out.printf("Warning: Settings3140/%s/%s entry %d does not equal default value of %d.\n",
                        subsystem, entry, m_current_value, default_value);
            }
            m_table_entry.setInteger(m_current_value);
            m_table_entry.setPersistent();
            setter.run(m_current_value);
        }

        @Override
        void checkValue() {
            int value = 0;

            // check the program value first
            value = m_getter.run();
            if (value != m_current_value) {
                m_table_entry.setInteger(value);
                m_current_value = value;
            }

            // check the networktable second
            value = (int) m_table_entry.getInteger(m_current_value);
            if (value != m_current_value) {
                m_setter.run(value);
                m_current_value = value;
            }
        }
    }

    private class BooleanUpdater extends SettingUpdater {

        final BooleanGetter m_getter;
        final BooleanSetter m_setter;
        boolean m_current_value = false;
        final NetworkTableEntry m_table_entry;

        BooleanUpdater(String subsystem, String entry, BooleanGetter getter, BooleanSetter setter,
                boolean default_value,
                TimePeriod period) {
            this.m_subsystem = subsystem;
            this.m_entry = entry;
            this.m_getter = getter;
            this.m_setter = setter;
            this.m_period = period;

            m_table_entry = m_settings_table.getSubTable(subsystem).getEntry(entry);
            m_current_value = m_table_entry.getBoolean(default_value);
            if (m_current_value != default_value) {
                System.out.printf("Warning: Settings3140/%s/%s entry %s does not equal default value of %s.\n",
                        subsystem, entry, m_current_value, default_value);
            }
            m_table_entry.setBoolean(m_current_value);
            m_table_entry.setPersistent();
            setter.run(m_current_value);
        }

        @Override
        void checkValue() {
            boolean value = false;

            // check the program value first
            value = m_getter.run();
            if (value != m_current_value) {
                m_table_entry.setBoolean(value);
                m_current_value = value;
            }

            // check the networktable second
            value = m_table_entry.getBoolean(m_current_value);
            if (value != m_current_value) {
                m_setter.run(value);
                m_current_value = value;
            }
        }
    }

    private class StringUpdater extends SettingUpdater {

        final StringGetter m_getter;
        final StringSetter m_setter;
        String m_current_value = null;
        final NetworkTableEntry m_table_entry;

        StringUpdater(String subsystem, String entry, StringGetter getter, StringSetter setter,
                String default_value,
                TimePeriod period) {
            this.m_subsystem = subsystem;
            this.m_entry = entry;
            this.m_getter = getter;
            this.m_setter = setter;
            this.m_period = period;

            m_table_entry = m_settings_table.getSubTable(subsystem).getEntry(entry);
            m_current_value = m_table_entry.getString(default_value);
            if (m_current_value != default_value) {
                System.out.printf("Warning: Settings3140/%s/%s entry %s does not equal default value of %s.\n",
                        subsystem, entry, m_current_value, default_value);
            }
            m_table_entry.setString(m_current_value);
            m_table_entry.setPersistent();
            setter.run(m_current_value);
        }

        @Override
        void checkValue() {
            String value = null;

            // check the program value first
            value = m_getter.run();
            if (value != m_current_value) {
                m_table_entry.setString(value);
                m_current_value = value;
            }

            // check the networktable second
            value = m_table_entry.getString(m_current_value);
            if (value != m_current_value) {
                m_setter.run(value);
                m_current_value = value;
            }
        }
    }

    private static Comms3140 s_global_instance = null;

    NetworkTable m_settings_table = NetworkTableInstance.getDefault().getTable("Settings3140");
    NetworkTable m_telemetry_table = NetworkTableInstance.getDefault().getTable("Telemetry3140");

    ArrayList<SettingUpdater> m_update_list = new ArrayList<SettingUpdater>();

    // public static final int kEveryPeriodic = 0;
    // public static final int kEveryOneSeconds = 1;
    // public static final int kEveryFiveSeconds = 2;
    // public static final int kEveryTenSeconds = 3;

    enum TimePeriod {
        kEveryPeriodic, kEveryOneSeconds, kEveryTenSeconds
    }

    String m_last_error = "";
    String m_last_warning = "";

    String m_telemetry_ip_address = "broadcast";
    int m_telemetry_udp_port = 13140;
    boolean m_telemetry_udp_enabled = false;

    /**
     * Get a global instace of the comms object.
     */
    public static Comms3140 getInstance() {

        if (s_global_instance == null) {
            s_global_instance = new Comms3140();
        }

        assert s_global_instance != null;
        return s_global_instance;
    }

    private Comms3140() {
        m_settings_table = NetworkTableInstance.getDefault().getTable("Settings3140");
        m_telemetry_table = NetworkTableInstance.getDefault().getTable("Telemetry3140");

        // registerDoubleSetting("Test", "Tmp", () -> tmp, (t) -> {
        // tmp = t;
        // }, 0.0, TimePeriod.kEveryPeriodic);
        registerStringSetting("Telemetry", "Telemetry IP Address", () -> m_telemetry_ip_address, (value) -> {
            m_telemetry_ip_address = value;
        }, m_telemetry_ip_address);
        registerIntSetting("Telemetry", "Telemetry UDP Port", () -> m_telemetry_udp_port, (value) -> {
            m_telemetry_udp_port = value;
        }, m_telemetry_udp_port);
        registerBooleanSetting("Telemetry", "Telemetry UDP Enabled", () -> m_telemetry_udp_enabled, (value) -> {
            m_telemetry_udp_enabled = value;
        }, m_telemetry_udp_enabled);

    }

    /**
     * This will check the current setting in the network table and overwrite
     * the value if needed. If the value changes the network table will be updated.
     * 
     * @param subsystem     A string representing the subsystem
     * @param entry         A string representing the setting entry name
     * @param getter        can be this::getValue or a lambda function () -> value
     * @param setter        can be this::setValue or a lambda function (v) -> {value
     *                      = v;}
     * @param default_value The default value for the setting. 0.0 is default.
     * @param update_period How often the value is checked. Default
     *                      TimePeriod.kEveryOneSeconds.
     */
    public void registerDoubleSetting(String subsystem, String entry, DoubleGetter getter, DoubleSetter setter,
            double default_value, TimePeriod update_period) {
        DoubleUpdater updater = new DoubleUpdater(subsystem, entry, getter, setter, default_value, update_period);
        m_update_list.add(updater);
    }

    public void registerDoubleSetting(String subsystem, String entry, DoubleGetter getter, DoubleSetter setter,
            double default_value) {
        registerDoubleSetting(subsystem, entry, getter, setter,
                default_value, TimePeriod.kEveryOneSeconds);
    }

    public void registerDoubleSetting(String subsystem, String entry, DoubleGetter getter, DoubleSetter setter,
            TimePeriod update_period) {
        registerDoubleSetting(subsystem, entry, getter, setter,
                0.0, update_period);
    }

    public void registerDoubleSetting(String subsystem, String entry, DoubleGetter getter, DoubleSetter setter) {
        registerDoubleSetting(subsystem, entry, getter, setter,
                0.0, TimePeriod.kEveryOneSeconds);
    }

    /**
     * This will check the current setting in the network table and overwrite
     * the value if needed. If the value changes the network table will be updated.
     * 
     * @param subsystem     A string representing the subsystem
     * @param entry         A string representing the setting entry name
     * @param getter        can be this::getValue or a lambda function () -> value
     * @param setter        can be this::setValue or a lambda function (v) -> {value
     *                      = v;}
     * @param default_value The default value for the setting. 0 is default.
     * @param update_period How often the value is checked. Default
     *                      TimePeriod.kEveryOneSeconds.
     */
    public void registerIntSetting(String subsystem, String entry, IntGetter getter, IntSetter setter,
            int default_value, TimePeriod update_period) {
        IntUpdater updater = new IntUpdater(subsystem, entry, getter, setter, default_value, update_period);
        m_update_list.add(updater);
    }

    public void registerIntSetting(String subsystem, String entry, IntGetter getter, IntSetter setter,
            int default_value) {
        registerIntSetting(subsystem, entry, getter, setter,
                default_value, TimePeriod.kEveryOneSeconds);
    }

    public void registerIntSetting(String subsystem, String entry, IntGetter getter, IntSetter setter,
            TimePeriod update_period) {
        registerIntSetting(subsystem, entry, getter, setter,
                0, update_period);
    }

    public void registerIntSetting(String subsystem, String entry, IntGetter getter, IntSetter setter) {
        registerIntSetting(subsystem, entry, getter, setter,
                0, TimePeriod.kEveryOneSeconds);
    }

    /**
     * This will check the current setting in the network table and overwrite
     * the value if needed. If the value changes the network table will be updated.
     * 
     * @param subsystem     A string representing the subsystem
     * @param entry         A string representing the setting entry name
     * @param getter        can be this::getValue or a lambda function () -> value
     * @param setter        can be this::setValue or a lambda function (v) -> {value
     *                      = v;}
     * @param default_value The default value for the setting. false is default.
     * @param update_period How often the value is checked. Default
     *                      TimePeriod.kEveryOneSeconds.
     */
    public void registerBooleanSetting(String subsystem, String entry, BooleanGetter getter, BooleanSetter setter,
            boolean default_value, TimePeriod update_period) {
        BooleanUpdater updater = new BooleanUpdater(subsystem, entry, getter, setter, default_value, update_period);
        m_update_list.add(updater);
    }

    public void registerBooleanSetting(String subsystem, String entry, BooleanGetter getter, BooleanSetter setter,
            TimePeriod update_period) {
        registerBooleanSetting(subsystem, entry, getter, setter,
                false, update_period);
    }

    public void registerBooleanSetting(String subsystem, String entry, BooleanGetter getter, BooleanSetter setter,
            boolean default_value) {
        registerBooleanSetting(subsystem, entry, getter, setter,
                default_value, TimePeriod.kEveryOneSeconds);
    }

    public void registerBooleanSetting(String subsystem, String entry, BooleanGetter getter, BooleanSetter setter) {
        registerBooleanSetting(subsystem, entry, getter, setter, false, TimePeriod.kEveryOneSeconds);
    }

    /**
     * This will check the current setting in the network table and overwrite
     * the value if needed. If the value changes the network table will be updated.
     * 
     * @param subsystem     A string representing the subsystem
     * @param entry         A string representing the setting entry name
     * @param getter        can be this::getValue or a lambda function () -> value
     * @param setter        can be this::setValue or a lambda function (v) -> {value
     *                      = v;}
     * @param default_value The default value for the setting. "" is default.
     * @param update_period How often the value is checked. Default
     *                      TimePeriod.kEveryOneSeconds.
     */
    public void registerStringSetting(String subsystem, String entry, StringGetter getter, StringSetter setter,
            String default_value, TimePeriod update_period) {
        StringUpdater updater = new StringUpdater(subsystem, entry, getter, setter, default_value, update_period);
        m_update_list.add(updater);
    }

    public void registerStringSetting(String subsystem, String entry, StringGetter getter, StringSetter setter,
            String default_value) {
        registerStringSetting(subsystem, entry, getter, setter, default_value, TimePeriod.kEveryOneSeconds);
    }

    public void registerStringSetting(String subsystem, String entry, StringGetter getter, StringSetter setter,
            TimePeriod period) {
        registerStringSetting(subsystem, entry, getter, setter, "", period);
    }

    public void registerStringSetting(String subsystem, String entry, StringGetter getter, StringSetter setter) {
        registerStringSetting(subsystem, entry, getter, setter, "", TimePeriod.kEveryOneSeconds);
    }

    @Override
    public void periodic() {
        super.periodic();

        double current_time = 1000.0 * System.currentTimeMillis();

        // Syncronize the variables
        for (SettingUpdater each : m_update_list) {
            each.update(current_time);
        }
    }

    // Send and update to the network table
    public void sendDoubleTelemetry(String subsystem, String entry, double value){
        m_telemetry_table.getSubTable(subsystem).getEntry(entry).setDouble(value);
    }

    // Send and update to the network table
    public void sendIntegerTelemetry(String subsystem, String entry, int value){
        m_telemetry_table.getSubTable(subsystem).getEntry(entry).setInteger(value);
    }

    // Send and update to the network table
    public void sendStringTelemetry(String subsystem, String entry, String value){
        m_telemetry_table.getSubTable(subsystem).getEntry(entry).setString(value);
    }

    // Send and update to the network table
    public void sendBooleanTelemetry(String subsystem, String entry, boolean value){
        m_telemetry_table.getSubTable(subsystem).getEntry(entry).setBoolean(value);
    }

}
