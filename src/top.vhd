library ieee;
use ieee.std_logic_1164.ALL;
use ieee.std_logic_unsigned.all;
use ieee.numeric_std.all;

use work.sata_defines.all;
use work.transport_layer_pkg.all;

entity top is
    port(
--        cpu_rst_n : in std_logic;       -- CPU_RESETn pushbutton. (Debounce this). Pin

        pll_refclk_150 : in std_logic;  -- 150MHz PLL refclk for XCVR design. Pin

        rx_serial_data : in  std_logic; -- XCVR input serial line.
        tx_serial_data : out std_logic; -- XCVR output serial line

--        USER_PB_FPGA1  : in  std_logic; -- PB1, used as alternative reset so that we can reset without resetting the clock control circuits

        USER_SW_0      : in  std_logic; -- AA28_N?
        USER_SW_1      : in  std_logic; -- AB28_P?

        LED0_N_PL      : out std_logic; -- LED0, PIN_F6
        LED1_N_PL      : out std_logic; -- LED1, PIN_G7

-- ANIOS Connectors
        i2c_SCL        : inout std_logic; -- PIN_W21
        i2c_SDA        : inout std_logic; -- PIN_AA24

-- HPS Connectors.. needed?
        memory_mem_a                    : out   std_logic_vector(15 downto  0);
        memory_mem_ba                   : out   std_logic_vector(2  downto  0);
        memory_mem_ck                   : out   std_logic;
        memory_mem_ck_n                 : out   std_logic;
        memory_mem_cke                  : out   std_logic;
        memory_mem_cs_n                 : out   std_logic;
        memory_mem_ras_n                : out   std_logic;
        memory_mem_cas_n                : out   std_logic;
        memory_mem_we_n                 : out   std_logic;
        memory_mem_reset_n              : out   std_logic;
        memory_mem_dq                   : inout std_logic_vector(31 downto  0);
        memory_mem_dqs                  : inout std_logic_vector(3  downto  0);
        memory_mem_dqs_n                : inout std_logic_vector(3  downto  0);
        memory_mem_odt                  : out   std_logic;
        memory_mem_dm                   : out   std_logic_vector(3  downto  0);
        memory_oct_rzqin                : in    std_logic
        );
end top;

architecture top_arch of top is
    -- top level signals
    signal clk25            : std_logic;
    signal clk50            : std_logic;
    signal si_reset            : std_logic;
    signal reset            : std_logic;
    signal cold_reset_n     : std_logic;

    signal switch_0 : std_logic;
    signal switch_1 : std_logic;

    signal fabric_clk_37_5  : std_logic; -- use this to clock the receive datapath
    -- xcvr signals
    --PHY Control Signals
    signal rxclkout                : std_logic; -- use this to clock the receive datapath
    signal txclkout                : std_logic; -- use this to clock the transmit datapath
    signal pll_locked               : std_logic; -- is the pll reference clock locked in

    signal rx_data         : std_logic_vector(31 downto 0); -- received data
    signal tx_data         : std_logic_vector(31 downto 0); -- data to transmit

    signal tx_forceelecidle         : std_logic; -- force signal idle for OOB signaling
    signal rx_signaldetect          : std_logic; -- detect signal idle for OOB signaling
    signal rx_is_lockedtoref        : std_logic; -- receiver is locked to pll reference clock
    signal rx_is_lockedtodata       : std_logic; -- receiver is locked to CDR block from received data


    signal rx_pma_clkout            : std_logic; -- recovered clock from cdr circuitry.

    signal do_word_align            : std_logic; -- perform word alignment to the comma 28.5 character
    signal rx_patterndetect         : std_logic_vector(3 downto 0); -- are we detecting the comma character in received data?
    signal rx_syncstatus            : std_logic_vector(3 downto 0); -- are we synced to data? (why is this 4 bits wide...?)

    signal rx_errdetect             : std_logic_vector(3 downto 0); -- reports a 8B/10B Code violation
    signal rx_disperr               : std_logic_vector(3 downto 0); -- reports a 8B/10B Disparity Error

    signal tx_datak                 : std_logic_vector(3 downto 0); -- send control character on specified byte instead of data character (k28.5 instead of d28.5)
    signal rx_datak                 : std_logic_vector(3 downto 0); -- reports which bytes contained control characters

    signal tx_ready           : std_logic; -- is the transmitter ready
    signal rx_ready           : std_logic; -- is the receiver ready

    --rst signals
    signal pll_powerdown      : std_logic; --      pll_powerdown.pll_powerdown
    signal tx_analogreset     : std_logic; --     tx_analogreset.tx_analogreset
    signal tx_digitalreset    : std_logic; --    tx_digitalreset.tx_digitalreset
    signal rx_analogreset     : std_logic; --     rx_analogreset.rx_analogreset
    signal rx_digitalreset    : std_logic; --    rx_digitalreset.rx_digitalreset

    signal tx_cal_busy        : std_logic; --        tx_cal_busy.tx_cal_busy
    signal rx_cal_busy        : std_logic; --        rx_cal_busy.rx_cal_busy

    signal rx_set_locktodata  : std_logic;
    signal rx_set_locktoref   : std_logic;

    -- reconfig signals
    signal reconfig_from_xcvr       : std_logic_vector(91 downto 0);
    signal reconfig_to_xcvr         : std_logic_vector(139 downto 0);
    signal reconfig_busy            : std_logic;

    -- link/phy hookup signals
    signal phy_status_to_link       : std_logic_vector(PHY_STATUS_LENGTH-1 downto 0);
    signal link_status_to_phy       : std_logic_vector(LINK_STATUS_LENGTH-1 downto 0);
    signal tx_data_from_link        : std_logic_vector(31 downto 0);
    signal rx_data_to_link          : std_logic_vector(31 downto 0);

    signal trans_status_in          : std_logic_vector(7 downto 0);
    signal trans_status_out         : std_logic_vector(7 downto 0);
    signal trans_tx_data_in         : std_logic_vector(31 downto 0);
    signal trans_rx_data_out        : std_logic_vector(31 downto 0);
    signal rst_n                    : std_logic;

    --signal declarations for dummy application process
    signal user_cmd_to_trans : std_logic_vector(2 downto 0);
    signal clear_errors : std_logic;
    signal user_data_to_trans : std_logic_vector(DATA_WIDTH - 1 downto 0);
    signal user_address_to_trans : std_logic_vector(DATA_WIDTH - 1 downto 0);

    signal trans_status_to_user : std_logic_vector(5 downto 0);
    signal trans_data_to_user : std_logic_vector(DATA_WIDTH - 1 downto 0);
    signal trans_address_to_user : std_logic_vector(DATA_WIDTH - 1 downto 0);

    signal app_control_counter : integer range 0 to 1000001;
    signal app_data_counter : integer range 0 to BUFFER_DEPTH;

    signal logical_sector_index : integer range 0 to BUFFER_DEPTH;
    signal app_count_up : std_logic;

    signal msata_device_ready : std_logic;
    signal app_write_valid : std_logic;
    signal app_send_read_valid : std_logic;
    signal app_receive_read_valid : std_logic;
    signal test_write_address : std_logic_vector(DATA_WIDTH - 1 downto 0);
    signal app_read_sent : std_logic;
    signal error_counter : integer range 0 to integer'high;
    signal words_read    : integer range 0 to integer'high;

    signal user_cmd_to_trans_prev : std_logic_vector(2 downto 0);
    signal app_receive_read_valid_prev : std_logic;

    -- i2c refckl manager signals
    signal i2c_done                 : std_logic;
    signal i2c_error                : std_logic;

    component basic_hps is
        port (
            memory_mem_a       : out   std_logic_vector(15 downto 0);                    -- mem_a
            memory_mem_ba      : out   std_logic_vector(2 downto 0);                     -- mem_ba
            memory_mem_ck      : out   std_logic;                                        -- mem_ck
            memory_mem_ck_n    : out   std_logic;                                        -- mem_ck_n
            memory_mem_cke     : out   std_logic;                                        -- mem_cke
            memory_mem_cs_n    : out   std_logic;                                        -- mem_cs_n
            memory_mem_ras_n   : out   std_logic;                                        -- mem_ras_n
            memory_mem_cas_n   : out   std_logic;                                        -- mem_cas_n
            memory_mem_we_n    : out   std_logic;                                        -- mem_we_n
            memory_mem_reset_n : out   std_logic;                                        -- mem_reset_n
            memory_mem_dq      : inout std_logic_vector(31 downto 0) := (others => 'X'); -- mem_dq
            memory_mem_dqs     : inout std_logic_vector(3 downto 0)  := (others => 'X'); -- mem_dqs
            memory_mem_dqs_n   : inout std_logic_vector(3 downto 0)  := (others => 'X'); -- mem_dqs_n
            memory_mem_odt     : out   std_logic;                                        -- mem_odt
            memory_mem_dm      : out   std_logic_vector(3 downto 0);                     -- mem_dm
            memory_oct_rzqin   : in    std_logic                     := 'X';             -- oct_rzqin
            cold_reset_reset_n : out   std_logic;                                        -- reset_n
            clk_25_clk         : out   std_logic                                         -- clk
        );
    end component basic_hps;

    component transport_dummy is
        port(
                fabric_clk          :   in std_logic;
                reset               :   in std_logic;

                --Interface with link Layer
                trans_status_to_link:   out std_logic_vector(7 downto 0);  -- [FIFO_RDY/n, transmit request, data complete, escape, bad FIS, error, good FIS]
                link_status_to_trans:   in  std_logic_vector(7 downto 0);  -- [Link Idle, transmit bad status, transmit good status, crc good/bad, comm error, fail transmit]
                tx_data_to_link     :   out std_logic_vector(31 downto 0);
                rx_data_from_link   :   in  std_logic_vector(31 downto 0)
                );
    end component transport_dummy;

    component transport_layer is
        port(
            --Interface with Application Layer
            rst_n           :   in std_logic;
            clk         :   in std_logic;

            data_from_user      :   in std_logic_vector(DATA_WIDTH - 1 downto 0);
            address_from_user   :   in std_logic_vector(DATA_WIDTH - 1 downto 0);

            user_command            :   in std_logic_vector(2 downto 0);
            clear_errors            :   in std_logic;
            status_to_user          :   out std_logic_vector(5 downto 0);

            data_to_user       :   out std_logic_vector(DATA_WIDTH - 1 downto 0);
            address_to_user    :   out std_logic_vector(DATA_WIDTH - 1 downto 0);

            --Interface with Link Layer
            status_to_link :    out std_logic_vector(7 downto 0); --for test just use bit 0 to indicate data ready
            status_from_link     :   in std_logic_vector(7 downto 0);
            data_to_link     :   out std_logic_vector(DATA_WIDTH - 1 downto 0);
            data_from_link      :   in std_logic_vector(DATA_WIDTH - 1 downto 0));

    end component transport_layer;

    component link_layer_32bit is
    port(-- Input
            clk             :   in std_logic;
            rst_n           :   in std_logic;

            --Interface with Transport Layer
            trans_status_in :   in std_logic_vector(7 downto 0);        -- [FIFO_RDY/n, transmit request, data complete, escape, bad FIS, error, good FIS]
            trans_status_out:   out std_logic_vector(7 downto 0);       -- [Link Idle, transmit bad status, transmit good status, crc good/bad, comm error, fail transmit]
            tx_data_in      :   in std_logic_vector(31 downto 0);
            rx_data_out     :   out std_logic_vector(31 downto 0);

            --Interface with Physical Layer
            tx_data_out     :   out std_logic_vector(31 downto 0);
            rx_data_in      :   in std_logic_vector(31 downto 0);
            phy_status_in   :   in std_logic_vector(3 downto 0);        -- [primitive, PHYRDY/n, Dec_Err]
            phy_status_out  :   out std_logic_vector(1 downto 0);       -- [primitive, clear status signals]
            perform_init    :   out std_logic);
    end component;

    component phy_layer_32bit is
        port(
            fabric_clk_37_5 : in std_logic;
            reset         : in std_logic;

            --Interface with link layer
            tx_data_from_link:   in std_logic_vector(31 downto 0);
            rx_data_to_link  :   out std_logic_vector(31 downto 0);
            phy_status_to_link : out std_logic_vector(PHY_STATUS_LENGTH-1 downto 0);
            link_status_to_phy : in std_logic_vector(LINK_STATUS_LENGTH-1 downto 0);

            --Interface with transceivers
            rxclkout         : in std_logic;   -- recovered rx clock to clock receive datapath from XCVRs
            txclkout         : in  std_logic;  -- tx clock from XCVRs to clock transmit datapath
            rx_pma_clkout    : in std_logic;                      --           rx_pma_clkout.rx_pma_clkout

            rx_data : in  std_logic_vector(31 downto 0); --raw received data from XCVRs
            rx_datak         : in  std_logic_vector(3 downto 0); --data or control symbol for receieved data
            rx_signaldetect  : in  std_logic; -- detect oob received oob signals

            rx_errdetect     : in std_logic_vector(3 downto 0);

            tx_forceelecidle : out std_logic; -- send oob signals
            tx_data : out std_logic_vector(31 downto 0); -- parallel data to transmit
            tx_datak         : out std_logic_vector(3 downto 0); -- data or control symbol for transmitted data

            do_word_align    : out std_logic; -- signal native phy to perform word align
            rx_syncstatus    : in std_logic_vector(3 downto 0); -- detect word alignment successfull

            rx_set_locktoref  : out std_logic; -- control transceiver locking characteristics
            rx_set_locktodata : out std_logic -- control transceiver locking characteristics
            );
    end component phy_layer_32bit;

    component xcvr_native_phy is
        port (
            pll_powerdown           : in  std_logic; --           pll_powerdown.pll_powerdown
            tx_analogreset          : in  std_logic; --          tx_analogreset.tx_analogreset
            tx_digitalreset         : in  std_logic; --         tx_digitalreset.tx_digitalreset
            tx_pll_refclk           : in  std_logic; --           tx_pll_refclk.tx_pll_refclk
            tx_serial_data          : out std_logic;                      --          tx_serial_data.tx_serial_data
            pll_locked              : out std_logic;                      --              pll_locked.pll_locked
            rx_analogreset          : in  std_logic; --          rx_analogreset.rx_analogreset
            rx_digitalreset         : in  std_logic; --         rx_digitalreset.rx_digitalreset
            rx_cdr_refclk           : in  std_logic; --           rx_cdr_refclk.rx_cdr_refclk
            rx_pma_clkout           : out std_logic;                      --           rx_pma_clkout.rx_pma_clkout
            rx_serial_data          : in  std_logic; --          rx_serial_data.rx_serial_data
            rx_set_locktodata       : in  std_logic; --       rx_set_locktodata.rx_set_locktodata
            rx_set_locktoref        : in  std_logic; --        rx_set_locktoref.rx_set_locktoref
            rx_is_lockedtoref       : out std_logic;                      --       rx_is_lockedtoref.rx_is_lockedtoref
            rx_is_lockedtodata      : out std_logic;                      --      rx_is_lockedtodata.rx_is_lockedtodata
            tx_std_coreclkin        : in  std_logic; --        tx_std_coreclkin.tx_std_coreclkin
            rx_std_coreclkin        : in  std_logic; --        rx_std_coreclkin.rx_std_coreclkin
            tx_std_clkout           : out std_logic;                      --           tx_std_clkout.tx_std_clkout
            rx_std_clkout           : out std_logic;                      --           rx_std_clkout.rx_std_clkout
            rx_std_wa_patternalign  : in  std_logic; --  rx_std_wa_patternalign.rx_std_wa_patternalign
            tx_std_polinv           : in  std_logic; --           tx_std_polinv.tx_std_polinv
            rx_std_polinv           : in  std_logic; --           rx_std_polinv.rx_std_polinv
            tx_std_elecidle         : in  std_logic; --         tx_std_elecidle.tx_std_elecidle
            rx_std_signaldetect     : out std_logic;                      --     rx_std_signaldetect.rx_std_signaldetect
            tx_cal_busy             : out std_logic;                      --             tx_cal_busy.tx_cal_busy
            rx_cal_busy             : out std_logic;                      --             rx_cal_busy.rx_cal_busy
            reconfig_to_xcvr        : in  std_logic_vector(139 downto 0); --        reconfig_to_xcvr.reconfig_to_xcvr
            reconfig_from_xcvr      : out std_logic_vector(91 downto 0);                     --      reconfig_from_xcvr.reconfig_from_xcvr
            tx_parallel_data        : in  std_logic_vector(31 downto 0); --        tx_parallel_data.tx_parallel_data
            tx_datak                : in  std_logic_vector(3 downto 0); --                tx_datak.tx_datak
            unused_tx_parallel_data : in  std_logic_vector(7 downto 0); -- unused_tx_parallel_data.unused_tx_parallel_data
            rx_parallel_data        : out std_logic_vector(31 downto 0);                     --        rx_parallel_data.rx_parallel_data
            rx_datak                : out std_logic_vector(3 downto 0);                      --                rx_datak.rx_datak
            rx_errdetect            : out std_logic_vector(3 downto 0);                      --            rx_errdetect.rx_errdetect
            rx_disperr              : out std_logic_vector(3 downto 0);                      --              rx_disperr.rx_disperr
            rx_runningdisp          : out std_logic_vector(3 downto 0);                      --          rx_runningdisp.rx_runningdisp
            rx_patterndetect        : out std_logic_vector(3 downto 0);                      --        rx_patterndetect.rx_patterndetect
            rx_syncstatus           : out std_logic_vector(3 downto 0);                      --           rx_syncstatus.rx_syncstatus
            unused_rx_parallel_data : out std_logic_vector(7 downto 0)                       -- unused_rx_parallel_data.unused_rx_parallel_data
        );
    end component xcvr_native_phy;

    component xcvr_reset is
        port (
            clock              : in  std_logic;             --              clock.clk
            reset              : in  std_logic;             --              reset.reset
            pll_powerdown      : out std_logic;                    --      pll_powerdown.pll_powerdown
            tx_analogreset     : out std_logic;                    --     tx_analogreset.tx_analogreset
            tx_digitalreset    : out std_logic;                    --    tx_digitalreset.tx_digitalreset
            tx_ready           : out std_logic;                    --           tx_ready.tx_ready
            pll_locked         : in  std_logic; --         pll_locked.pll_locked
            pll_select         : in  std_logic; --         pll_select.pll_select
            tx_cal_busy        : in  std_logic; --        tx_cal_busy.tx_cal_busy
            rx_analogreset     : out std_logic;                    --     rx_analogreset.rx_analogreset
            rx_digitalreset    : out std_logic;                    --    rx_digitalreset.rx_digitalreset
            rx_ready           : out std_logic;                    --           rx_ready.rx_ready
            rx_is_lockedtodata : in  std_logic; -- rx_is_lockedtodata.rx_is_lockedtodata
            rx_cal_busy        : in  std_logic--        rx_cal_busy.rx_cal_busy
        );
    end component xcvr_reset;

    --component XCVR_CustomReset
    --    port (
    --        clk50              : in std_logic;
    --        master_reset       : in std_logic;
    --        pll_powerdown      : out std_logic;
    --        tx_digitalreset    : out std_logic;
    --        rx_analogreset     : out std_logic;
    --        rx_digitalreset    : out std_logic;
    --        rx_locktorefclk    : out std_logic;
    --        rx_locktodata      : out std_logic;
    --        busy               : in std_logic;
    --        pll_locked         : in std_logic;
    --        oob_handshake_done : in std_logic
    --    );
    --end component;

    component xcvr_reconf is
        port (
            reconfig_busy             : out std_logic;                                         --      reconfig_busy.reconfig_busy
            mgmt_clk_clk              : in  std_logic;             --       mgmt_clk_clk.clk
            mgmt_rst_reset            : in  std_logic;             --     mgmt_rst_reset.reset
            reconfig_mgmt_address     : in  std_logic_vector(6 downto 0)   ; --      reconfig_mgmt.address
            reconfig_mgmt_read        : in  std_logic;             --                   .read
            reconfig_mgmt_readdata    : out std_logic_vector(31 downto 0);                     --                   .readdata
            reconfig_mgmt_waitrequest : out std_logic;                                         --                   .waitrequest
            reconfig_mgmt_write       : in  std_logic;             --                   .write
            reconfig_mgmt_writedata   : in  std_logic_vector(31 downto 0); --                   .writedata
            reconfig_mif_address      : out std_logic_vector(31 downto 0);                     --       reconfig_mif.address
            reconfig_mif_read         : out std_logic;                                         --                   .read
            reconfig_mif_readdata     : in  std_logic_vector(15 downto 0); --                   .readdata
            reconfig_mif_waitrequest  : in  std_logic;             --                   .waitrequest
            reconfig_to_xcvr          : out std_logic_vector(139 downto 0);                    --   reconfig_to_xcvr.reconfig_to_xcvr
            reconfig_from_xcvr        : in  std_logic_vector(91 downto 0)  -- reconfig_from_xcvr.reconfig_from_xcvr
        );
    end component xcvr_reconf;

    component Debounce is
      port(
        clk50      : in  std_logic;
        button     : in  std_logic;
        debounced  : out std_logic);
    end component Debounce;


    component pll_25MHz_to_37_5MHz is
        port (
            refclk   : in  std_logic := '0'; --  refclk.clk
            rst      : in  std_logic := '0'; --   reset.reset
            outclk_0 : out std_logic;        -- outclk0.clk
            locked   : out std_logic         --  locked.export
        );
    end component pll_25MHz_to_37_5MHz;

    component pll_25MHz_to_50MHz is
        port (
            refclk   : in  std_logic := '0'; --  refclk.clk
            rst      : in  std_logic := '0'; --   reset.reset
            outclk_0 : out std_logic;        -- outclk0.clk
            locked   : out std_logic         --  locked.export
        );
    end component pll_25MHz_to_50MHz;

    component si5338 is
        GENERIC(
        input_clk       : INTEGER := 50_000_000; --input clock speed from user logic in Hz
         i2c_address    : std_logic_vector(6 downto 0) := "111" & "0000";
        bus_clk         : INTEGER := 400_000);   --speed the i2c bus (scl) will run at in Hz
        port
            (
                clk             : in std_logic;
                reset           : in std_logic;

                done            : out std_logic;
                error           : out std_logic;

                SCL             : inout std_logic;
                SDA             : inout std_logic
            );
    end component si5338;

    begin

    --i_transdummy1 : transport_dummy
    --port map(
    --        fabric_clk => txclkout,
    --        reset      => rst_n,
    --        trans_status_to_link => trans_status_in,
    --        link_status_to_trans => trans_status_out,
    --        tx_data_to_link      => trans_tx_data_in,
    --        rx_data_from_link    => trans_rx_data_out
    --    );
    i_transport_layer1 : transport_layer
        port map(

            clk => txclkout,
            rst_n      => rst_n,

            --Interface with Application Layer
            data_from_user => user_data_to_trans,
            address_from_user => user_address_to_trans,

            user_command => user_cmd_to_trans,
            clear_errors => clear_errors,
            status_to_user => trans_status_to_user,

            data_to_user => trans_data_to_user,
            address_to_user => trans_address_to_user,

            --Interface with Link Layer
            status_to_link => trans_status_in,

            status_from_link => trans_status_out,
            data_to_link => trans_tx_data_in,
            data_from_link => trans_rx_data_out
            );


    i_linkLayer1 : link_layer_32bit
    port map(   -- Input
            clk             => txclkout,
            rst_n           => rst_n,

            --Interface with Transport Layer
            trans_status_in => trans_status_in,
            trans_status_out=> trans_status_out,
            tx_data_in      => trans_tx_data_in,
            rx_data_out     => trans_rx_data_out,

            --Interface with Physical Layer
            tx_data_out     => tx_data_from_link,
            rx_data_in      => rx_data_to_link,
            phy_status_in   => phy_status_to_link,
            phy_status_out  => link_status_to_phy
--            perform_init    => perform_init
        );

    i_phy_layer_1 : phy_layer_32bit
    port map(
            fabric_clk_37_5 => txclkout,
            reset         => reset,

            --Interface with link layer
            tx_data_from_link    => tx_data_from_link,
            rx_data_to_link      => rx_data_to_link,
            phy_status_to_link   => phy_status_to_link,
            link_status_to_phy   => link_status_to_phy,
    --        perform_init     :   out std_logic); -- currently unused

            --Interface with transceivers
            rxclkout         => rxclkout,
            txclkout         => txclkout,
            rx_pma_clkout    => rx_pma_clkout,

            rx_data          => rx_data,
            rx_datak         => rx_datak,
            rx_signaldetect  => rx_signaldetect,

            rx_errdetect     => rx_errdetect,

            tx_forceelecidle => tx_forceelecidle,
            tx_data          => tx_data,
            tx_datak         => tx_datak,

            do_word_align    => do_word_align,
            rx_syncstatus    => rx_syncstatus,

            rx_set_locktoref  => rx_set_locktoref,
            rx_set_locktodata => rx_set_locktodata
        );

    native1 : xcvr_native_phy
        port  map(
            pll_powerdown           => pll_powerdown,
            tx_analogreset          => tx_analogreset,
            tx_digitalreset         => tx_digitalreset,
            tx_pll_refclk           => pll_refclk_150,
            tx_serial_data          => tx_serial_data,
            pll_locked              => pll_locked,
            rx_analogreset          => rx_analogreset,
            rx_digitalreset         => rx_digitalreset,
            rx_cdr_refclk           => pll_refclk_150, -- cdr refclk is same as pll refclk!!!
            rx_pma_clkout           => rx_pma_clkout,
            rx_serial_data          => rx_serial_data,
            rx_set_locktodata       => rx_set_locktodata,
            rx_set_locktoref        => rx_set_locktoref,
            rx_is_lockedtoref       => rx_is_lockedtoref,
            rx_is_lockedtodata      => rx_is_lockedtodata,
            tx_std_coreclkin        => txclkout,
            rx_std_coreclkin        => rxclkout,
            tx_std_clkout           => txclkout,
            rx_std_clkout           => rxclkout,
            rx_std_wa_patternalign  => do_word_align,

            tx_std_polinv           => '0',
            rx_std_polinv           => '1', -- because the p/n pairs are reversed during soldering on the rx side, need to reverse the polarity manually.

            tx_std_elecidle         => tx_forceelecidle,
            rx_std_signaldetect     => rx_signaldetect,
            tx_cal_busy             => tx_cal_busy,
            rx_cal_busy             => rx_cal_busy,
            reconfig_to_xcvr        => reconfig_to_xcvr,
            reconfig_from_xcvr      => reconfig_from_xcvr,
            tx_parallel_data        => tx_data,
            tx_datak                => tx_datak,
            unused_tx_parallel_data => (others => '0'), -- don't need the unused parallel data
            rx_parallel_data        => rx_data,
            rx_datak                => rx_datak,
            rx_errdetect            => rx_errdetect, -- code violation or disparity error
            rx_disperr              => rx_disperr,   -- disparity error only
            rx_runningdisp          => OPEN, -- we don't care about what the current running disparity is.
            rx_patterndetect        => rx_patterndetect, -- word alignment detected the comma pattern, k28.5
            rx_syncstatus           => rx_syncstatus -- syncstatus high indicates word aligned
--            unused_rx_parallel_data => -- don't need the unused parallel data
       );

    reconf_1 : xcvr_reconf
        port map (
            reconfig_busy             => reconfig_busy,
            mgmt_clk_clk              => clk25,
            mgmt_rst_reset            => si_reset,
            reconfig_mgmt_address     => (others => '0'),
            reconfig_mgmt_read        => '0',
            --reconfig_mgmt_readdata
            --reconfig_mgmt_waitrequest
            reconfig_mgmt_write       => '0',
            reconfig_mgmt_writedata   => (others => '0'),
--            reconfig_mif_address      : out std_logic_vector(31 downto 0);                     --       reconfig_mif.address
--            reconfig_mif_read         : out std_logic;                                         --                   .read
            reconfig_mif_readdata     => (others => '0'),
            reconfig_mif_waitrequest  => '0',
            reconfig_to_xcvr          => reconfig_to_xcvr,
            reconfig_from_xcvr        => reconfig_from_xcvr
        );

    --customRst1 : XCVR_CustomReset
    --    port map (
    --        clk50              => clk50,
    --        master_reset       => reset,
    --        pll_powerdown      => pll_powerdown,
    --        tx_digitalreset    => tx_digitalreset,
    --        rx_analogreset     => rx_analogreset,
    --        rx_digitalreset    => rx_digitalreset,
    --        rx_locktorefclk    => rx_set_locktoref,
    --        rx_locktodata      => rx_set_locktodata,
    --        busy               => tx_cal_busy,
    --        pll_locked         => pll_locked,
    --        oob_handshake_done => oob_handshake_done
    --    );


    xcvr_reset1 : xcvr_reset
        port map (
            clock              => clk50,
            reset              => si_reset,
            pll_powerdown      => pll_powerdown,
            tx_analogreset     => tx_analogreset,
            tx_digitalreset    => tx_digitalreset,
            tx_ready           => tx_ready,
            pll_locked         => pll_locked,
            pll_select         => '0',
            tx_cal_busy        => tx_cal_busy,
            rx_analogreset     => rx_analogreset,
            rx_digitalreset    => rx_digitalreset,
            rx_ready           => rx_ready,
            rx_is_lockedtodata => rx_is_lockedtodata,
            rx_cal_busy        => rx_cal_busy
        );

    i_pll_25MHz_to_37_5MHz_1 : pll_25MHz_to_37_5MHz
        port map(
            refclk   => clk25,
            rst      => '0',
            outclk_0 => fabric_clk_37_5
--            locked   : out std_logic         --  locked.export
        );

    i_pll_25MHz_to_50_MHz_1 : pll_25MHz_to_50MHz
        port map(
            refclk   => clk25,
            rst      => '0',
            outclk_0 => clk50
--            locked   : out std_logic         --  locked.export
        );

    i_switch_debounce_0 : Debounce
        port map(clk50, USER_SW_0, switch_0);

    i_switch_debounce_1 : Debounce
        port map(clk50, USER_SW_1, switch_1);

    si_reset <= not switch_0;
    reset <= si_reset; --or not pll_locked;
    rst_n <= not reset;

    LED0_N_PL <= '0' when switch_0 = '1' else 'Z';
    LED1_N_PL <= '0' when switch_1 = '1' or (error_counter = 1000000 and words_read = 1000000) else 'Z';

    --Instantiate I2C component
    i_si5338_1 : si5338
        generic map(
            input_clk       => 50_000_000, --input clock speed from user logic in Hz
            i2c_address     => "111" & "0000",
            bus_clk         => 400_000    --speed the i2c bus (scl) will run at in Hz
        )
        port map(
            clk             => clk50,
            reset           => si_reset,

            done            => i2c_done,
            error           => i2c_error,

            SCL             => i2c_scl,--IO_B5A_RX_R6_W21_P,  --SI5338_SCL, -- BLUE CHANNEL -- D4
            SDA             => i2c_sda --IO_B5A_RX_R9_AA24_P--SI5338_SDA --GREEN CHANNEL -- D6
    );

    p0 : component basic_hps
        port map (
            memory_mem_a                    => memory_mem_a,
            memory_mem_ba                   => memory_mem_ba,
            memory_mem_ck                   => memory_mem_ck,
            memory_mem_ck_n                 => memory_mem_ck_n,
            memory_mem_cke                  => memory_mem_cke,
            memory_mem_cs_n                 => memory_mem_cs_n,
            memory_mem_ras_n                => memory_mem_ras_n,
            memory_mem_cas_n                => memory_mem_cas_n,
            memory_mem_we_n                 => memory_mem_we_n,
            memory_mem_reset_n              => memory_mem_reset_n,
            memory_mem_dq                   => memory_mem_dq,
            memory_mem_dqs                  => memory_mem_dqs,
            memory_mem_dqs_n                => memory_mem_dqs_n,
            memory_mem_odt                  => memory_mem_odt,
            memory_mem_dm                   => memory_mem_dm,
            memory_oct_rzqin                => memory_oct_rzqin,
            clk_25_clk                      => clk25,
            cold_reset_reset_n              => cold_reset_n
        );
--dummy process to act as user application
    user_application  :   process(txclkout, rst_n)
    begin
        if(rst_n = '0')then
            user_cmd_to_trans <= "000";
            user_data_to_trans <= (others => '0');
            user_address_to_trans <= (others => '0');
            app_data_counter <= 0;
            app_control_counter <= 0;
            app_read_sent <= '0';
            logical_sector_index <= 0;
            app_count_up <= '1';
            error_counter <= 0;
            words_read <= 0;
            user_cmd_to_trans_prev <= "000";
            app_receive_read_valid_prev <= '0';
        elsif(rising_edge(txclkout))then
            user_cmd_to_trans_prev <= user_cmd_to_trans;
            app_receive_read_valid_prev <= app_receive_read_valid;
            if(msata_device_ready = '1')then
                if(app_control_counter < (2 * 2 * BUFFER_DEPTH))then --send write
                    if(app_write_valid = '1')then
                        if(app_data_counter < BUFFER_DEPTH)then
                            user_cmd_to_trans <= "001";--send write
                            --user_address_to_trans <= test_write_address;
                            user_address_to_trans <= std_logic_vector(to_unsigned(logical_sector_index,DATA_WIDTH));
                            if(app_count_up = '1')then
                                user_data_to_trans <= std_logic_vector(to_unsigned(app_data_counter,DATA_WIDTH));
                            else
                                user_data_to_trans <= std_logic_vector(to_unsigned(BUFFER_DEPTH - 1 - app_data_counter,DATA_WIDTH));
                            end if;
                            app_data_counter <= app_data_counter + 1;
                        else
                            user_cmd_to_trans <= "000";
                            user_data_to_trans <= (others => '1');
                            user_address_to_trans <= (others => '1');
                        end if;
                        app_control_counter <= app_control_counter + 1;
                    else
                        app_control_counter <= app_control_counter;
                    end if;
                elsif(app_control_counter < 4 * 2 * BUFFER_DEPTH)then --send read
                    app_data_counter <= 0;
                    if(app_send_read_valid = '1' and app_read_sent = '0')then
                        user_cmd_to_trans <= "010";--send read
                        user_address_to_trans <= std_logic_vector(to_unsigned(logical_sector_index,DATA_WIDTH));
                        user_data_to_trans <= (others => '1');
                        app_read_sent <= '1';
                    elsif(app_read_sent = '1')then
                        user_cmd_to_trans <= "000";
                        if(app_write_valid = '1' or app_send_read_valid = '1')then
                            app_control_counter <= app_control_counter + 1;
                        end if;
                    else
                        app_control_counter <= app_control_counter;
                    end if;
                elsif(app_control_counter < 6 * 2 * BUFFER_DEPTH)then --retrieve read
                    if(app_receive_read_valid = '1')then
                        user_cmd_to_trans <= "100";
                        user_address_to_trans <= std_logic_vector(to_unsigned(logical_sector_index,DATA_WIDTH));
                        user_data_to_trans <= (others => '1');
                    else
                        user_cmd_to_trans <= "000";
                    end if;

                    if(user_cmd_to_trans_prev = "100" and app_receive_read_valid_prev = '1') then
                        if(app_data_counter < BUFFER_DEPTH)then
                            app_data_counter <= app_data_counter + 1;
                        end if;
                        --something <= trans_data_to_user;
                        if (app_count_up = '1') then
                            if(trans_data_to_user /= std_logic_vector(to_unsigned(app_data_counter,DATA_WIDTH))) then
                                error_counter <= error_counter + 1;
                            end if;
                        else -- down
                            if(trans_data_to_user /= std_logic_vector(to_unsigned(BUFFER_DEPTH - 1 - app_data_counter,DATA_WIDTH))) then
                                error_counter <= error_counter + 1;
                            end if;
                        end if;
                        words_read <= words_read + 1;
                    end if;

                    app_control_counter <= app_control_counter + 1;
                elsif(app_control_counter >= 8 * 2 * BUFFER_DEPTH)then --reset
                    user_cmd_to_trans <= "000";
                    user_data_to_trans <= (others => '0');
                    user_address_to_trans <= (others => '0');
                    if(logical_sector_index < BUFFER_DEPTH)then
                        logical_sector_index <= logical_sector_index + BUFFER_DEPTH/DWORDS_PER_SECTOR;
                    else
                        logical_sector_index <= 0;
                        app_count_up <= not app_count_up;
                    end if;
                    app_control_counter <= 0;
                    app_data_counter <= 0;
                    app_read_sent <= '0';
                else --wait and increment
                    user_cmd_to_trans <= "000";
                    user_data_to_trans <= (others => '0');
                    user_address_to_trans <= (others => '0');
                    app_control_counter <= app_control_counter + 1;
                end if;
            end if;
        end if;
    end process;

    msata_device_ready <= trans_status_to_user(0);
    app_write_valid <= trans_status_to_user(1);
    app_send_read_valid <= trans_status_to_user(2);
    app_receive_read_valid <= trans_status_to_user(3);

    test_write_address <= (others  => '0'); --remove this to allow address functionaity
    clear_errors <= '1';

end top_arch;
