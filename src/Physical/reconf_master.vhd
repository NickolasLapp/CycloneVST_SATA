library ieee;
use ieee.std_logic_1164.ALL;
use ieee.std_logic_unsigned.all;
use ieee.numeric_std.all;

use work.sata_defines.all;

entity reconf_master is
    port(
            reconfig_busy             : in  std_logic;
            mgmt_clk_clk              : out std_logic;
            mgmt_rst_reset            : out std_logic;
            -- control signals
            reconfig_start            : in  std_logic;
            reconfig_done             : out std_logic;
            -- reconfig signals
            reconfig_mgmt_address     : out std_logic_vector(6 downto 0);
            reconfig_mgmt_read        : out std_logic;
            reconfig_mgmt_readdata    : in  std_logic_vector(31 downto 0);
            reconfig_mgmt_waitrequest : in  std_logic;
            reconfig_mgmt_write       : out std_logic;
            reconfig_mgmt_writedata   : out std_logic_vector(31 downto 0);

            reconfig_mif_address      : in  std_logic_vector(31 downto 0);
            reconfig_mif_read         : in  std_logic;
            reconfig_mif_readdata     : out std_logic_vector(15 downto 0);
            reconfig_mif_waitrequest  : out std_logic
        );
    );
end entity reconf_master;

architecture rtl of reconf_master is

    constant LOGICAL_CHANNEL_ADR    : std_logic_vector(6 downto 0) := x"38";
    constant STREAMER_CSR_ADR       : std_logic_vector(6 downto 0) := x"3A";
    constant STREAMER_OFFSET_ADR    : std_logic_vector(6 downto 0) := x"3B";
    constant STREAMER_DATA_ADR      : std_logic_vector(6 downto 0) := x"3C";

    type RECONFIG_MASTER_STATE is  (S0_RECONFIG_IDLE,
                                    S1_WRITE_LOGICAL_CHANNEL,
                                    S2_WRITE_MIF_MODE_TO_CSR,
                                    S3_WRITE_MIF_BASE_ADDRESS_TO_OFFSET,
                                    S4_WRITE_MIF_BASE_ADDRESS_TO_DATA,
                                    S5_WRITE_WRITE_START_TO_CSR,
                                    S6_WRITE_WRITE_START_TO_OFFSET,
                                    S7_WRITE_START_STREAM_TO_DATA,
                                    S8_WRITE_INITIATE_STREAM_TO_CSR,
                                    S9_READ_WAIT_BUSY,
                                    S10_RECONFIG_DONE);
    signal reconfig_state : RECONFIG_MASTER_STATE;
    signal reconfig_next_state : RECONFIG_MASTER_STATE;


    signal reconfig_mif_readdata;
    signal reconfig_mif_waitrequest;

    component to_3000Mbps_rom is
    port(
        clock       : in std_logic;
        address     : in std_logic_vector(3 downto 0);
        q           : out std_logic_vector(31 downto 0)
    );
    end component;


    begin

    NEXT_STATE_LOGIC : process(reconfig_state)
    begin
        case reconfig_state
            when S0_RECONFIG_IDLE                       =>
                if(reconfig_start = '1') then
                    reconfig_next_state <= S1_WRITE_LOGICAL_CHANNEL;
                else
                    reconfig_next_state <= S0_RECONFIG_IDLE;
                end if;
            when S1_WRITE_LOGICAL_CHANNEL               =>
                reconfig_next_state <= S2_WRITE_MIF_MODE_TO_CSR;
            when S2_WRITE_MIF_MODE_TO_CSR               =>
                reconfig_next_state <= S3_WRITE_MIF_BASE_ADDRESS_TO_OFFSET;
            when S3_WRITE_MIF_BASE_ADDRESS_TO_OFFSET    =>
                reconfig_next_state <= S4_WRITE_MIF_BASE_ADDRESS_TO_DATA;
            when S4_WRITE_MIF_BASE_ADDRESS_TO_DATA      =>
                reconfig_next_state <= S5_WRITE_WRITE_START_TO_CSR;
            when S5_WRITE_WRITE_START_TO_CSR            =>
                reconfig_next_state <= S6_WRITE_WRITE_START_TO_OFFSET;
            when S6_WRITE_WRITE_START_TO_OFFSET         =>
                reconfig_next_state <= S7_WRITE_START_STREAM_TO_DATA;
            when S7_WRITE_START_STREAM_TO_DATA          =>
                reconfig_next_state <= S8_WRITE_INITIATE_STREAM_TO_CSR;
            when S8_WRITE_INITIATE_STREAM_TO_CSR        =>
                reconfig_next_state <= S9_READ_WAIT_BUSY;
            when S9_READ_WAIT_BUSY                      =>
                reconfig_next_state <= S10_RECONFIG_DONE;
            when S10_RECONFIG_DONE
                if(reconfig_start = '1') then
                    reconfig_next_state <= S1_WRITE_LOGICAL_CHANNEL;
                else
                    reconfig_next_state <= S10_RECONFIG_DONE;
                end if;
            when others                                 =>
                reconfig_next_state <= S0_RECONFIG_IDLE;
        end case;
    end process;

    STATE_UPDATE_LOGIC : process(mgmt_clk_clk, mgmt_rst_reset)
    begin
    if(mgmt_rst_reset = '1') then
        reconfig_state <= S1_WRITE_LOGICAL_CHANNEL;
    elsif(rising_edge(mgmt_clk_clk)) then
        if(reconfig_state = S9_READ_WAIT_BUSY) then
            if(reconfig_mgmt_readdata(8) = '0') then -- busy deasserted
                reconfig_state <= reconfig_next_state;
            end if;
        elsif(reconfig_state = S10_RECONFIG_DONE) then
            reconfig_state <= reconfig_next_state
        else--if(reconfig_state = *_WRITE_*) then
            if(reconfig_mgmt_waitrequest = '0') then -- only proceed if command completed by slave
                reconfig_state <= reconfig_next_state;
            end if;
        end if;
    end if;
    end process;

    OUTPUT_LOGIC : process(mgmt_clk_clk, mgmt_rst_reset)
    begin
        if(mgmt_rst_reset = '1') then
            reconfig_mgmt_address       <= (others => '0');
            reconfig_mgmt_write         <= '0';
            reconfig_mgmt_writedata     <= (others => '0');
            reconfig_mgmt_read          <= '0';
            reconfig_done               <= '0';
        elsif(rising_edge(mgmt_clk_clk)) then
            case reconfig_state is
                when S0_RECONFIG_IDLE                       =>
                    reconfig_mgmt_address       <= (others => '0');
                    reconfig_mgmt_write         <= '0';
                    reconfig_mgmt_writedata     <= (others => '0');
                    reconfig_mgmt_read          <= '0';
                    reconfig_done               <= '0';

                when S1_WRITE_LOGICAL_CHANNEL               => --1. Write the logical channel number to the Streamer logical channel register.
                    reconfig_mgmt_address       <= LOGICAL_CHANNEL_ADR;
                    reconfig_mgmt_write         <= '1';
                    reconfig_mgmt_writedata     <= (others => '0');
                    reconfig_mgmt_read          <= '0';

                when S2_WRITE_MIF_MODE_TO_CSR               => --2. Write MIF mode, 2’b00, to the Streamer control and status register mode bits.
                    reconfig_mgmt_address       <= STREAMER_CSR_ADR;
                    reconfig_mgmt_write         <= '1';
                    reconfig_mgmt_writedata     <= (others => '0');
                    reconfig_mgmt_read          <= '0';

                when S3_WRITE_MIF_BASE_ADDRESS_TO_OFFSET    => --3. Write the MIF base address, 0x0, to the Streamer offset register.
                    reconfig_mgmt_address       <= STREAMER_OFFSET_ADR;
                    reconfig_mgmt_write         <= '1';
                    reconfig_mgmt_writedata     <= (others => '0');
                    reconfig_mgmt_read          <= '0';

                when S4_WRITE_MIF_BASE_ADDRESS_TO_DATA      => --4. Write the base address of the MIF file to the Streamer data register.
                    reconfig_mgmt_address       <= STREAMER_DATA_ADR;
                    reconfig_mgmt_write         <= '1';
                    reconfig_mgmt_writedata     <= (others => '0');
                    reconfig_mgmt_read          <= '0';

                when S5_WRITE_WRITE_START_TO_CSR            => --5. Write the Streamer control and status register write bit to 1'b1
                                                               --to initiate a write of all the data set in the previous steps.
                    reconfig_mgmt_address       <= STREAMER_CSR_ADR;
                    reconfig_mgmt_write         <= '1';
                    reconfig_mgmt_writedata     <= x"00000001";
                    reconfig_mgmt_read          <= '0';

                when S6_WRITE_WRITE_START_TO_OFFSET         => --6. Write to the Streamer offset register with the value to start a MIF stream, 0x1.
                    reconfig_mgmt_address       <= STREAMER_OFFSET_ADR;
                    reconfig_mgmt_write         <= '1';
                    reconfig_mgmt_writedata     <= x"00000001";
                    reconfig_mgmt_read          <= '0';

                when S7_WRITE_START_STREAM_TO_DATA          => --7. Write the Streamer internal data register with the value 0x3 to setup the sxtreaming of the MIF.
                    reconfig_mgmt_address       <= STREAMER_DATA_ADR;
                    reconfig_mgmt_write         <= '1';
                    reconfig_mgmt_writedata     <= x"00000003";
                    reconfig_mgmt_read          <= '0';

                when S8_WRITE_INITIATE_STREAM_TO_CSR        => --8. Write to the Streamer control and status register to 1'b1, to initiate the streaming operation
                    reconfig_mgmt_address       <= STREAMER_CSR_ADR;
                    reconfig_mgmt_write         <= '1';
                    reconfig_mgmt_writedata     <= x"00000001";
                    reconfig_mgmt_read          <= '0';

                when S9_READ_WAIT_BUSY                      =>
                    reconfig_mgmt_address       <= STREAMER_CSR_ADR;
                    reconfig_mgmt_write         <= '0';
                    reconfig_mgmt_writedata     <= (others => '0');
                    reconfig_mgmt_read          <= '1';

                when S10_RECONFIG_DONE                      =>
                    reconfig_mgmt_address       <= (others => '0');
                    reconfig_mgmt_write         <= '0';
                    reconfig_mgmt_writedata     <= (others => '0');
                    reconfig_mgmt_read          <= '0';
                    reconfig_done               <= '1';

                when others                                 =>
                    reconfig_mgmt_address       <= (others => '0');
                    reconfig_mgmt_write         <= '0';
                    reconfig_mgmt_writedata     <= (others => '0');
                    reconfig_mgmt_read          <= '0';
            end case;
        end if;
    end process;


    -- MIF Signals
    -- to_3000Mbps is instantiated as a rom to be read by the reconfiguration controller.
    i_to_3000Mbps_rom_1 : to_3000Mbps_rom
        port map(
            clock   => mgmt_clk_clk,
            address => reconfig_mif_address,
            q       => reconfig_mif_readdata
        );

    -- waitrequest should be asserted at all times unless the reconfiguration controller just requested a read from the rom.
    -- In this case, becasue the rom is guaranteed to provide data at the next clock cycle, waitrequest should be deasserted.
    -- To accomplish this, we buffer the read signal by a clock edge, invert it, and pass it out to the device
    process(mgmt_clk_clk, mgmt_rst_reset)
    begin
        if(mgmt_rst_reset = '1') then
            reconfig_mif_waitrequest <= '1';
        elsif(rising_edge(mgmt_clk_clk)) then
            reconfig_mif_waitrequest <= not reconfig_mif_read;
        end if;
    end process;

end architecture rtl;