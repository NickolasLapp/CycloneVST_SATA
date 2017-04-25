library ieee;                   --! Use standard library.
use ieee.std_logic_1164.all;    --! Use standard logic elements
use ieee.numeric_std.all;       --! Use numeric standard

use work.sata_defines.all;
use work.transport_layer_pkg.all;
----------------------------------------------------------------------------
--Status Truth Table:
-- XXX0 == Device Not Ready
-- XXX1 == Device Ready
-- XX01 == Write Not Ready
-- XX11 == Write Ready
-- X0X1 == Send Read Not Ready
-- X1X1 == Send Ready Ready
-- 0XX1 == Retrieve Read Not Ready
-- 1XX1 == Retrieve Read Ready
--

--Command truth table
-- 000 == Do Nothing
-- X01 == Send Write     (Command to write data at specified address in SSD)
-- X10 == Send Read      (Command to retrieve data at specified address from SSD)
-- 1XX == Retrieve Read  (Command to read value from Rx buffer)
----------------------------------------------------------------------------
entity transport_layer is
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
        status_to_link :    out std_logic_vector(7 downto 0);
        status_from_link     :   in std_logic_vector(7 downto 0);
        data_to_link     :   out std_logic_vector(DATA_WIDTH - 1 downto 0);
        data_from_link      :   in std_logic_vector(DATA_WIDTH - 1 downto 0));

end transport_layer;

architecture transport_layer_arch of transport_layer is
--States for Transport FSM

  signal current_state, next_state : State_Type;

    --======================================================================================
    --Signals to create Register Host to Device FIS contents
    signal fis_type : std_logic_vector(7 downto 0);

    --Shadow Registers... Somewhat customized for ease of use
    signal feature : std_logic_vector(15 downto 0); -- a reserved field in DMA read ext, DMA write ext. Set to all zeros
    signal lba : std_logic_vector(47 downto 0);   --address to write to / read from
    signal control : std_logic_vector(7 downto 0);  --Field not defined for DMA read/write ext. Thus is "reserved", set to zeros
    signal command : std_logic_vector(7 downto 0);  --35h for dma write ext, 25h dma read ext
    signal c_bit       : std_logic;                 --Set to one when register transfer is due to update of command reg.
    signal count : std_logic_vector(15 downto 0);   --# of logical sectors to be transferred for DMA. 0000h indicates 65.536 sectors --not currently using
    --------------------------------------------------
    --  set bit 6 to 1, bit 4 is Transport Dependent, think it should be zero
    --Bits 7, 5 are obsolete? Currently planning on setting to zero
    signal device: std_logic_vector(7 downto 0);
    --------------------------------------------------
    signal i_bit        : std_logic;                    --used only for device to host
    signal status       : std_logic_vector(7 downto 0); --used only for device to host
    signal error        : std_logic_vector(7 downto 0); --used only for device to host
    --======================================================================================

    signal error_address : std_logic_vector(DATA_WIDTH - 1 downto 0);

    signal last_read_address : data_width_array_type;

    signal command_error, read_error, write_error : std_logic;

    signal tx_fis_array, rx_fis_array   :   register_fis_array_type; -- signals to hold host to device register FIS contents

    signal transport_ready : std_logic;

    --from link interface status signals
    signal link_rdy, pause, data_from_link_valid, link_error : std_logic;

    --to link interface status signals
    signal tx_to_link_request : std_logic;
    signal rx_from_link_ready : std_logic;

    signal paused_data_to_link : std_logic_vector(DATA_WIDTH - 1 downto 0);

    signal link_fis_type : std_logic_vector(7 downto 0);



    constant DEVICE_READY_INDEX         : integer := 0;
    constant WRITE_VALID_INDEX          : integer := 1;
    constant SEND_READ_VALID_INDEX      : integer := 2;
    constant RETRIEVE_READ_VALID_INDEX  : integer := 3;
    constant WRITE_ERROR_INDEX          : integer := 4;
    constant READ_ERROR_INDEX           : integer := 5;

    constant STATUS_ERR : integer := 16;--index of error bit in first word of register device to host FIS
    constant STATUS_DF : integer := 21; --index of device fault bit in first word of register device to host FIS
    --constant STATUS_BSY : integer := 0; --not checking busy bit, testing indicates it is uncessary

component buffer_2prt_ram is
    port
    (
        clock       : in std_logic  := '1';
        data        : in std_logic_vector (31 downto 0);
        rdaddress       : in std_logic_vector (10 downto 0);
        wraddress       : in std_logic_vector (10 downto 0);
        wren        : in std_logic  := '0';
        q       : out std_logic_vector (31 downto 0)
    );
end component buffer_2prt_ram;

    --======================================================================================
    --Buffers
    signal tx_buf_data_in, tx_buf_data_out : data_width_array_type;
    signal tx_wren : std_logic_vector(1 downto 0);
    signal rx_buf_data_in, rx_buf_data_out : data_width_array_type;
    signal rx_wren : std_logic_vector(1 downto 0);
    signal tx_buffer_full, rx_buffer_full, tx_buffer_empty, rx_buffer_empty   : std_logic_vector(1 downto 0);
    signal tx0_locked, tx1_locked, rx0_locked, rx1_locked : std_logic; -- signal to allow SM to take control of buffers
    signal tx_index : integer range 0 to 1; -- signal to use as index to array of tx register FISs
    signal rx_index : integer range 0 to 1; -- signal to use as index to array of rx register FISs

    signal tx_write_addr, rx_write_addr : data_width_array_type;
    signal tx_read_addr, rx_read_addr : std_logic_vector (DATA_WIDTH - 1 downto 0);

    signal tx_write_ptr, tx_read_ptr : integer range 0 to BUFFER_DEPTH + 1;
    signal rx_write_ptr, rx_read_ptr : integer range 0 to BUFFER_DEPTH + 1;

    signal tx_buffer : double_buffer;
    signal rx_buffer : double_buffer;

    signal rx_buffer_read_select : integer range 0 to 1;
    signal s_data_to_link : std_logic_vector (DATA_WIDTH - 1 downto 0);
    signal tx_buffer_data_valid : std_logic;
    signal rx_buffer_data_valid : std_logic;
    --======================================================================================

begin
    tx_buffer_0 : buffer_2prt_ram
        port map(
                    clock => clk,
                    data => tx_buf_data_in(0),
                    rdaddress => tx_read_addr(10 downto 0),--Quartus IP only defines 11 address bits because RAM depth is fixed
                    wraddress => tx_write_addr(0)(10 downto 0),--Quartus IP only defines 11 address bits because RAM depth is fixed
                    wren => tx_wren(0),
                    q => tx_buf_data_out(0)
            );
    tx_buffer_1 : buffer_2prt_ram
        port map(
                    clock => clk,
                    data => tx_buf_data_in(1),
                    rdaddress => tx_read_addr(10 downto 0),--Quartus IP only defines 11 address bits because RAM depth is fixed
                    wraddress => tx_write_addr(1)(10 downto 0),--Quartus IP only defines 11 address bits because RAM depth is fixed
                    wren => tx_wren(1),
                    q => tx_buf_data_out(1)
            );
    rx_buffer_0 : buffer_2prt_ram
        port map(
                    clock => clk,
                    data => rx_buf_data_in(0),
                    rdaddress => rx_read_addr(10 downto 0),--Quartus IP only defines 11 address bits because RAM depth is fixed
                    wraddress => rx_write_addr(0)(10 downto 0),--Quartus IP only defines 11 address bits because RAM depth is fixed
                    wren => rx_wren(0),
                    q => rx_buf_data_out(0)
            );
    rx_buffer_1 : buffer_2prt_ram
        port map(
                    clock => clk,
                    data => rx_buf_data_in(1),
                    rdaddress => rx_read_addr(10 downto 0),--Quartus IP only defines 11 address bits because RAM depth is fixed
                    wraddress => rx_write_addr(1)(10 downto 0),--Quartus IP only defines 11 address bits because RAM depth is fixed
                    wren => rx_wren(1),
                    q => rx_buf_data_out(1)
            );

--=================================================================================================================
--Transport Layer Finite State Machine
--=================================================================================================================
    transport_state_memory  :   process(clk, rst_n)
      begin
        if (rst_n = '0') then
            current_state <= transport_reset;
        elsif (rising_edge(clk)) then
            if (pause = '0') then
                current_state <= next_state;
            end if;
        end if;
    end process;

    transport_next_state_logic: process (current_state, status_from_link, link_rdy, data_from_link,link_fis_type, user_command,rst_n,pause, tx_index,
                                         rx_index, tx_buffer_full, rx_buffer_full, tx_read_ptr, data_from_link_valid, read_error, write_error, link_error)
      begin
        case (current_state) is
        ----------------------------------------------- -----------------------------------------------
            -- Idle SM states (top level)
        ----------------------------------------------- -----------------------------------------------
            when transport_reset =>
                if (rst_n = '0') then
                    next_state <= transport_reset;
                else
                    next_state <= transport_init_start;
                end if;
            when transport_init_start =>
                if (data_from_link(7 downto 0) = REG_DEVICE_TO_HOST) then
                    next_state <= transport_init_end;
                else
                    next_state <= transport_init_start;
                end if;
            when transport_init_end =>
                if (link_error = '1') then --error occurred. Loop back to init_start, because need status to come through for device to work
                    next_state <= transport_init_start;
                elsif (data_from_link_valid = '1') then    --link layer is sending other fields of the initial status FIS, but information is not useful
                    next_state <= transport_init_end;
                else                                    --link layer finished transmitting initial status FIS
                    next_state <= identify_device_0;
                end if;
            when identify_device_0    =>
                if (link_rdy = '1' and pause = '0') then --link layer has accepted data word, move to next state
                    next_state <= identify_device_1;
                else
                    next_state <= identify_device_0;
                end if;
            when identify_device_1    =>
                if (link_rdy = '1' and pause = '0') then --link layer has accepted data word, move to next state
                    next_state <= identify_device_2;
                else
                    next_state <= identify_device_1;
                end if;
            when identify_device_2    =>
                if (link_rdy = '1' and pause = '0') then --link layer has accepted data word, move to next state
                    next_state <= identify_device_3;
                else
                    next_state <= identify_device_2;
                end if;
            when identify_device_3    =>
                if (link_rdy = '1' and pause = '0') then --link layer has accepted data word, move to next state
                    next_state <= identify_device_4;
                else
                    next_state <= identify_device_3;
                end if;
            when identify_device_4    =>
                if (link_rdy = '1' and pause = '0') then --link layer has accepted data word, move to next state
                    next_state <= rx_pio_setup;
                else
                    next_state <= identify_device_4;
                end if;
            when rx_pio_setup =>
                if(link_error = '1') then --device did not correctly receive identify_device fis, must retransmit to ensure valid operation
                    next_state <= identify_device_0;
                elsif (link_fis_type = PIO_SETUP_FIS) then --wait for PIO Setup FIS, a protocol command that comes before the identify device packet
                    next_state <= rx_identify_packet;
                else
                    next_state <= rx_pio_setup;
                end if;
            when rx_identify_packet => --wait until data fis is being transmitted
                if (link_fis_type = DATA_FIS) then
                    next_state <= wait_for_fis_end; --currently not processing the identify device packet, because this project is only targetting a single SSD with known characteristics
                else
                    next_state <= rx_identify_packet;
                end if;
            when wait_for_fis_end =>
                if (data_from_link_valid = '1') then
                    next_state <= wait_for_fis_end;
                elsif (link_error = '1' or read_error = '1' or write_error = '1') then
                    next_state <= report_error;
                else
                    next_state <= transport_idle;
                end if;
            when transport_idle =>
                if (tx_buffer_full(0) = '1') then
                    next_state <= dma_write_idle;
                elsif (tx_buffer_full(1) = '1') then
                    next_state <= dma_write_idle;
                elsif (user_command(1 downto 0) = "10") then
                    next_state <= dma_read_idle;
                else
                    next_state <= transport_idle;
                end if;
--========================================================================================
                -- DMA Write EXT SM states
            when dma_write_idle     =>
                next_state <= dma_write_reg_fis_0;
            when dma_write_reg_fis_0    =>
                if (link_rdy = '1' and pause = '0') then
                    next_state <= dma_write_reg_fis_1;
                else
                    next_state <= dma_write_reg_fis_0;
                end if;
            when dma_write_reg_fis_1    =>
                if (link_rdy = '1' and pause = '0') then
                    next_state <= dma_write_reg_fis_2;
                else
                    next_state <= dma_write_reg_fis_1;
                end if;
            when dma_write_reg_fis_2    =>
                if (link_rdy = '1' and pause = '0') then
                    next_state <= dma_write_reg_fis_3;
                else
                    next_state <= dma_write_reg_fis_2;
                end if;
            when dma_write_reg_fis_3    =>
                if (link_rdy = '1' and pause = '0') then
                    next_state <= dma_write_reg_fis_4;
                else
                    next_state <= dma_write_reg_fis_3;
                end if;
            when dma_write_reg_fis_4    =>
                if (link_rdy = '1' and pause = '0') then
                    next_state <= dma_write_chk_activate;
                else
                    next_state <= dma_write_reg_fis_4;
                end if;
            when dma_write_chk_activate =>
                if (link_fis_type = DMA_ACTIVATE_FIS) then
                    next_state <= dma_write_data_fis;
                elsif ((data_from_link (7 downto 0) = REG_DEVICE_TO_HOST and (data_from_link(STATUS_ERR) = '1' or data_from_link(STATUS_DF) = '1')) or link_error = '1') then
                    next_state <= report_error;
                else
                    next_state <= dma_write_chk_activate;
                end if;
            when dma_write_data_fis =>
                if (pause = '0' and link_rdy = '1') then
                    next_state <= dma_write_data_frame;
                else
                    next_state <= dma_write_data_fis;
                end if;
            when dma_write_data_frame   =>
                if (tx_read_ptr < BUFFER_DEPTH and link_rdy = '1') then
                    next_state <= dma_write_data_frame;
                else
                    next_state <= dma_write_chk_status;
                end if;
            when dma_write_chk_status   =>
                if (link_error = '1') then
                    next_state <= report_error;
                elsif (link_fis_type = REG_DEVICE_TO_HOST) then
                    if (data_from_link(STATUS_ERR) = '1' or data_from_link(STATUS_DF) = '1') then
                        next_state <= report_error;
                    else
                        next_state <= wait_for_fis_end;
                    end if;
                else
                    next_state <= dma_write_chk_status;
                end if;
--========================================================================================
            -- DMA Read EXT SM states
            when dma_read_idle      =>
                    next_state <= dma_read_reg_fis_0;
            when dma_read_reg_fis_0 =>
                if (link_rdy = '1' and pause = '0') then
                    next_state <= dma_read_reg_fis_1;
                else
                    next_state <= dma_read_reg_fis_0;
                end if;
            when dma_read_reg_fis_1 =>
                if (link_rdy = '1' and pause = '0') then
                    next_state <= dma_read_reg_fis_2;
                else
                    next_state <= dma_read_reg_fis_1;
                end if;
            when dma_read_reg_fis_2 =>
                if (link_rdy = '1' and pause = '0') then
                    next_state <= dma_read_reg_fis_3;
                else
                    next_state <= dma_read_reg_fis_2;
                end if;
            when dma_read_reg_fis_3 =>
                if (link_rdy = '1' and pause = '0') then
                    next_state <= dma_read_reg_fis_4;
                else
                    next_state <= dma_read_reg_fis_3;
                end if;
            when dma_read_reg_fis_4 =>
                if (link_rdy = '1' and pause = '0') then
                    next_state <= dma_read_data_fis;
                else
                    next_state <= dma_read_reg_fis_4;
                end if;
            when dma_read_data_fis  =>
                if (data_from_link(7 downto 0)= DATA_FIS) then
                    next_state <= dma_read_data_frame;
                elsif ((data_from_link (7 downto 0) = REG_DEVICE_TO_HOST and (data_from_link(STATUS_ERR) = '1' or data_from_link(STATUS_DF) = '1')) or link_error = '1') then --this condition is possible if address is out of range, or there is a device fault or media error
                    next_state <= report_error;
                else
                    next_state <= dma_read_data_fis;
                end if;
            when dma_read_data_frame    =>
                if (rx_buffer_full(rx_index) = '0' and data_from_link_valid = '1') then
                    next_state <= dma_read_data_frame;
                else
                    next_state <= dma_read_chk_status;
                end if;
            when dma_read_chk_status    =>
                if (link_fis_type = REG_DEVICE_TO_HOST) then
                    next_state <= wait_for_fis_end;
                else
                    next_state <= dma_read_chk_status;
                end if;
            when report_error =>
                next_state <= transport_idle;
--=======================================================================================
            when others =>  next_state <= transport_idle;
        end case;
    end process;
--=================================================================================================================
    transport_output_logic: process(clk,rst_n)
      begin
        if (rst_n = '0') then
            rx0_locked <= '0';
            rx1_locked <= '0';
            tx0_locked <= '0';
            tx1_locked <= '0';
            rx_from_link_ready <= '0';
            tx_to_link_request <= '0';
            tx_buffer_empty <= "11";
            rx_buffer_full <= "00";
            rx_wren <= "00";
            tx_read_ptr <= 0;
            rx_write_ptr <= 0;
            tx_index <= 0;
            transport_ready <= '0';
            s_data_to_link <= (others => '1');
            tx_buffer_data_valid <= '0';
            rx_buf_data_in <= (others => (others => '0'));
            rx_write_addr <= (others => (others => '0'));
            last_read_address <= (others => (others => '1'));
            error_address <= (others => '1');
            command_error <= '0';
            read_error <= '0';
            write_error <= '0';
        elsif (rising_edge(clk)) then
            if (pause = '0') then
                case (current_state) is
                ----------------------------------------------- -----------------------------------------------
                    -- Idle SM states (top level)
                ----------------------------------------------- -----------------------------------------------
                    when transport_reset =>
                        rx0_locked <= '0';
                        rx1_locked <= '0';
                        tx0_locked <= '0';
                        tx1_locked <= '0';
                        rx_from_link_ready <= '0';
                        tx_to_link_request <= '0';
                        tx_buffer_empty <= "11";
                        rx_buffer_full <= "00";
                        rx_wren <= "00";
                        tx_read_ptr <= 0;
                        rx_write_ptr <= 0;
                        tx_index <= 0;
                        transport_ready <= '0';
                        s_data_to_link <= (others => '1');
                        tx_buffer_data_valid <= '0';
                        rx_buf_data_in <= (others => (others => '0'));
                        rx_write_addr <= (others => (others => '0'));
                        last_read_address <= (others => (others => '0'));
                        error_address <= (others => '1');
                        command_error <= '0';
                        read_error <= '0';
                        write_error <= '0';
                    when transport_init_start =>
                        rx_from_link_ready <= '1';
                        tx_fis_array(tx_index).fis_type <= REG_HOST_TO_DEVICE;
                        tx_fis_array(tx_index).crrr_pm <= x"80"; --80 sets "c" bit, required
                        tx_fis_array(tx_index).command <= IDENTIFY_DEVICE;
                        tx_fis_array(tx_index).features <= x"00";
                        tx_fis_array(tx_index).lba <= (others => '0');
                        tx_fis_array(tx_index).device <= x"E0";
                        tx_fis_array(tx_index).features_ext <= x"00";
                        tx_fis_array(tx_index).lba_ext <= (others => '0');
                        tx_fis_array(tx_index).count <= (others => '0'); --on most drives 1 logical sector := 512 bytes
                        tx_fis_array(tx_index).icc <= x"00";
                        tx_fis_array(tx_index).control <= x"00";
                        tx_fis_array(tx_index).aux <= x"00000000";
                    when transport_init_end =>
                        rx_from_link_ready <= '1';
                        s_data_to_link <= (others => '1');
                    when identify_device_0 =>
                        rx_from_link_ready <= '0';
                        tx_to_link_request <= '1';
                        s_data_to_link <= tx_fis_array(tx_index).features & tx_fis_array(tx_index).command &
                                       tx_fis_array(tx_index).crrr_pm & tx_fis_array(tx_index).fis_type;
                    when identify_device_1 =>
                        s_data_to_link <= tx_fis_array(tx_index).device & tx_fis_array(tx_index).lba;
                    when identify_device_2 =>
                        s_data_to_link <= tx_fis_array(tx_index).features_ext &
                                        tx_fis_array(tx_index).lba_ext;
                    when identify_device_3 =>
                        s_data_to_link <= tx_fis_array(tx_index).control & tx_fis_array(tx_index).icc &
                                        tx_fis_array(tx_index).count;
                    when identify_device_4 =>
                        s_data_to_link <= tx_fis_array(tx_index).aux;
                    when rx_pio_setup =>
                        tx_to_link_request <= '0';
                        rx_from_link_ready <= '1';
                        s_data_to_link <= x"AAAAAAAA";
                    when rx_identify_packet =>
                        s_data_to_link <= x"BBBBBBBB";
                    when wait_for_fis_end =>
                        tx_to_link_request <= '0';
                        rx_from_link_ready <= '1';
                        s_data_to_link <= x"00BEEF00";
                    when transport_idle =>
                        transport_ready <= '1';
                        rx_from_link_ready <= '0';
                        tx_to_link_request <= '0';
                        s_data_to_link <= x"AAAAAAAA";
                        rx_wren <= "00";
                        if (rx_buffer_empty(0) = '1') then rx_buffer_full(0) <= '0'; end if;
                        if (rx_buffer_empty(1) = '1') then rx_buffer_full(1) <= '0'; end if;
                        if (tx_buffer_full(0) = '1') then
                            tx0_locked <= '1';
                            tx_buffer_empty(0) <= '0';
                            tx_index <= 0;
                        elsif (tx_buffer_full(1) = '1') then
                                tx1_locked <= '1';
                                tx_buffer_empty(1) <= '0';
                                tx_index <= 1;
                        elsif (user_command(1 downto 0) = "10") then
                            if (rx_buffer_empty(0) = '1') then
                                rx_index <= 0;
                                rx0_locked <= '1';
                            elsif (rx_buffer_empty(1) = '1') then
                                rx_index <= 1;
                                rx1_locked <= '1';
                            end if;
                        end if;
            ----------------------------------------------- --------------------------------------
        --========================================================================================
                    -- DMA Write EXT SM states
                    when dma_write_idle     =>
                        --build register host to device DMA Write FIS
                        tx_read_ptr <= 0;
                        tx_fis_array(tx_index).fis_type <= REG_HOST_TO_DEVICE;
                        tx_fis_array(tx_index).crrr_pm <= x"80"; --80 sets C bit
                        tx_fis_array(tx_index).command <= WRITE_DMA_EXT;
                        tx_fis_array(tx_index).features <= x"00";
                        tx_fis_array(tx_index).lba <= lba(23 downto 0);
                        tx_fis_array(tx_index).device <= x"E0";
                        tx_fis_array(tx_index).features_ext <= x"00";
                        tx_fis_array(tx_index).lba_ext <= lba(47 downto 24);
                        tx_fis_array(tx_index).count <= WRITE_SECTOR_COUNT; --on most drives 1 logical sector := 512 bytes
                        tx_fis_array(tx_index).icc <= x"00";
                        tx_fis_array(tx_index).control <= x"00";
                        tx_fis_array(tx_index).aux <= x"00000000";
                    when dma_write_reg_fis_0    =>
                        tx_to_link_request <= '1';
                        s_data_to_link <= tx_fis_array(tx_index).features & tx_fis_array(tx_index).command &
                                       tx_fis_array(tx_index).crrr_pm & tx_fis_array(tx_index).fis_type;
                    when dma_write_reg_fis_1    =>
                        s_data_to_link <= tx_fis_array(tx_index).device & tx_fis_array(tx_index).lba;
                    when dma_write_reg_fis_2    =>
                        s_data_to_link <= tx_fis_array(tx_index).features_ext &
                                       tx_fis_array(tx_index).lba_ext;
                    when dma_write_reg_fis_3    =>
                        s_data_to_link <= tx_fis_array(tx_index).control & tx_fis_array(tx_index).icc &
                                       tx_fis_array(tx_index).count;
                    when dma_write_reg_fis_4    =>
                        s_data_to_link <= tx_fis_array(tx_index).aux;
                    when dma_write_chk_activate =>
                        tx_to_link_request <= '0';
                        rx_from_link_ready <= '1';
                        s_data_to_link <= x"F0F0F0F0";
                        if ((data_from_link (7 downto 0) = REG_DEVICE_TO_HOST and (data_from_link(STATUS_ERR) = '1' or data_from_link(STATUS_DF) = '1')) or link_error = '1') then
                            error_address <= tx_fis_array(tx_index).lba_ext(7 downto 0) & tx_fis_array(tx_index).lba;
                            write_error <= '1';
                        end if;
                    when dma_write_data_fis =>
                        rx_from_link_ready <= '0';
                        tx_to_link_request <= '1';
                        s_data_to_link <=  x"000000" & DATA_FIS;
                    when dma_write_data_frame   =>
                        if (pause = '0') then
                            if (tx_read_ptr < BUFFER_DEPTH) then
                                tx_read_ptr <= tx_read_ptr + 1;
                                tx_buffer_data_valid <= '1';
                            else
                                tx_buffer_data_valid <= '0';
                                tx_to_link_request <= '0';
                                tx_buffer_empty(tx_index) <= '1';
                                if (tx_index = 0) then
                                    tx0_locked <= '0';
                                else
                                    tx1_locked <= '0';
                                end if;
                            end if;
                        end if;
                    when dma_write_chk_status   =>
                        rx_from_link_ready <= '1';
                        if (link_error = '1') then
                            error_address <= tx_fis_array(tx_index).lba_ext(7 downto 0) & tx_fis_array(tx_index).lba;
                            write_error <= '1';
                        elsif (data_from_link (7 downto 0) = REG_DEVICE_TO_HOST) then
                            if (data_from_link(STATUS_ERR) = '1' or data_from_link(STATUS_DF) = '1') then
                                error_address <= tx_fis_array(tx_index).lba_ext(7 downto 0) & tx_fis_array(tx_index).lba;
                                command_error <= '1';
                                write_error <= '1';
                            end if;
                        end if;
        --========================================================================================
                    -- DMA Read EXT SM states
                    when dma_read_idle      =>
                        rx_write_ptr <= 0;
                        --build register host to device DMA Read FIS
                        rx_fis_array(rx_index).fis_type <= REG_HOST_TO_DEVICE;
                        rx_fis_array(rx_index).crrr_pm <= x"80";
                        rx_fis_array(rx_index).command <= READ_DMA_EXT;
                        rx_fis_array(rx_index).features <= x"00";
                        rx_fis_array(rx_index).lba <= lba(23 downto 0);
                        rx_fis_array(rx_index).device <= x"E0";
                        rx_fis_array(rx_index).features_ext <= x"00";
                        rx_fis_array(rx_index).lba_ext <= lba(47 downto 24);
                        rx_fis_array(rx_index).count <= WRITE_SECTOR_COUNT; --on most drives 1 logical sector := 512 bytes
                        rx_fis_array(rx_index).icc <= x"00";
                        rx_fis_array(rx_index).control <= x"00";
                        rx_fis_array(rx_index).aux <= x"00000000";
                    when dma_read_reg_fis_0 =>
                        tx_to_link_request <= '1';
                        s_data_to_link <= rx_fis_array(rx_index).features & rx_fis_array(rx_index).command &
                                       rx_fis_array(rx_index).crrr_pm & rx_fis_array(rx_index).fis_type;
                    when dma_read_reg_fis_1 =>
                        s_data_to_link <= rx_fis_array(rx_index).device & rx_fis_array(rx_index).lba;
                    when dma_read_reg_fis_2 =>
                        s_data_to_link <= rx_fis_array(rx_index).features_ext &
                                       rx_fis_array(rx_index).lba_ext;
                    when dma_read_reg_fis_3 =>
                        s_data_to_link <= rx_fis_array(rx_index).control & rx_fis_array(rx_index).icc &
                                       rx_fis_array(rx_index).count;
                    when dma_read_reg_fis_4 =>
                        s_data_to_link <= rx_fis_array(rx_index).aux;
                    when dma_read_data_fis  =>
                        tx_to_link_request <= '0';
                        rx_from_link_ready <= '1';
                        if ((data_from_link (7 downto 0) = REG_DEVICE_TO_HOST and (data_from_link(STATUS_ERR) = '1' or data_from_link(STATUS_DF) = '1')) or link_error = '1') then
                            error_address <= rx_fis_array(rx_index).lba_ext(7 downto 0) & rx_fis_array(rx_index).lba;
                            read_error <= '1';
                        end if;
                    when dma_read_data_frame    =>
                        if (pause = '0') then
                            if (data_from_link_valid = '1') then
                                rx_wren(rx_index) <= '1';
                                rx_write_addr(rx_index) <= std_logic_vector(to_unsigned(rx_write_ptr,DATA_WIDTH));
                                rx_buf_data_in(rx_index) <= data_from_link;
                                if (rx_write_ptr < BUFFER_DEPTH - 1) then
                                    rx_write_ptr <= rx_write_ptr + 1;
                                else
                                    rx_buffer_full(rx_index) <= '1';
                                    rx_wren(rx_index) <= '0';
                                    if (rx_index = 0) then
                                        rx0_locked <= '0';
                                    else
                                        rx1_locked <= '0';
                                    end if;
                                end if;
                            end if;
                        end if;
                    when dma_read_chk_status =>
                        last_read_address(rx_index) <= rx_fis_array(rx_index).lba_ext(7 downto 0) & rx_fis_array(rx_index).lba;
                        rx_from_link_ready <= '1';
                        if (link_error = '1') then
                            error_address <= rx_fis_array(rx_index).lba_ext(7 downto 0) & rx_fis_array(rx_index).lba;
                            read_error <= '1';
                        elsif (data_from_link (7 downto 0) = REG_DEVICE_TO_HOST) then
                            if (data_from_link(STATUS_ERR) = '1' or data_from_link(STATUS_DF) = '1') then
                                error_address <= rx_fis_array(rx_index).lba_ext(7 downto 0) & rx_fis_array(rx_index).lba;
                                command_error <= '1';
                                read_error <= '1';
                            end if;
                        end if;
                    when report_error =>
                        if (clear_errors = '1') then
                            read_error <= '0';
                            write_error <= '0';
                            command_error <= '0';
                        elsif (write_error = '1') then
                            error_address <= tx_fis_array(tx_index).lba_ext(7 downto 0) & tx_fis_array(tx_index).lba;
                        elsif (read_error = '1') then
                            error_address <= rx_fis_array(rx_index).lba_ext(7 downto 0) & rx_fis_array(rx_index).lba;
                        --else --initialization error
                        end if;
        --========================================================================================
                    when others => -- state <= transport_idle;
                end case;
            end if;
        end if;
    end process;

--=================================================================================================================
--logic and processes to handle tx read RAM access
tx_buffer_reader : process (rst_n, pause, tx_read_ptr, tx_buffer_data_valid)
  begin
    if (rst_n = '0') then
        tx_read_addr <= (others => '0');
    elsif (tx_read_ptr < BUFFER_DEPTH and tx_buffer_data_valid = '1') then
        if (pause = '0') then
            tx_read_addr <= std_logic_vector(to_unsigned(tx_read_ptr,DATA_WIDTH));
        else
            tx_read_addr <= std_logic_vector(to_unsigned(tx_read_ptr-1,DATA_WIDTH));
        end if;
    else
        tx_read_addr <= (others => '0');
    end if;
end process;

switch_data_to_link : process(rst_n, pause, s_data_to_link, tx_buf_data_out,tx_buffer_data_valid, tx_index)
  begin
    if (rst_n = '0') then
        data_to_link <= (others => '0');
    elsif (tx_buffer_data_valid = '1') then
        data_to_link <= tx_buf_data_out(tx_index);
    else
        data_to_link <= s_data_to_link;
    end if;
end process;
--=================================================================================================================
--Processes to control the flow of user data to/from the tx/rx buffers
--The dual-buffer system allows user data to be written to a buffer even when the Transort FSM is performing a command
 tx_buffer_control   : process(clk,rst_n)
    variable tx_w_buffer : integer range 0 to 1;
    variable tx_wrt_ptr_var : integer range 0 to BUFFER_DEPTH;
    variable user_write_valid : std_logic;
      begin
        if (rst_n = '0') then
            tx_write_ptr <= 0;
            tx_wrt_ptr_var := 0;
            tx_w_buffer := 0;
            user_write_valid := '0';
            tx_buffer_full(0) <= '0';
            tx_buffer_full(1) <= '0';
            tx_wren <= "00";
            tx_write_addr <= (others => (others => '0'));
            tx_buf_data_in <= (others => (others => '0'));
            lba <= (others => '0');
        elsif (rising_edge(clk)) then

            if (tx_buffer_full(0) = '0' and tx0_locked = '0') then
                tx_w_buffer := 0;
                user_write_valid := '1';
                tx_wren(0) <= '1';
                tx_wren(1) <= '0';
            elsif (tx_buffer_full(1) = '0' and tx1_locked = '0') then
                tx_w_buffer := 1;
                user_write_valid := '1';
                tx_wren(0) <= '0';
                tx_wren(1) <= '1';
            else
                user_write_valid := '0';
                tx_wren <= "00";
            end if;

            if (tx_read_ptr > 0) then
                if (tx0_locked = '1') then
                    tx_buffer_full(0) <= '0';
                elsif (tx1_locked = '1') then
                    tx_buffer_full(1) <= '0';
                end if;
            end if;

            if (user_command(1 downto 0) = "01" and user_write_valid = '1') then --user is sending data
                if (tx_write_ptr < BUFFER_DEPTH - 1) then
                    tx_write_ptr <= tx_write_ptr + 1;
                    tx_wrt_ptr_var := tx_wrt_ptr_var + 1;
                else
                    tx_write_ptr <= 0;
                    tx_wrt_ptr_var := 0;
                    tx_buffer_full(tx_w_buffer) <= '1';
                end if;
                lba <= x"0000" &  address_from_user; --currently expecting user to keep address on line for entire write command
                tx_write_addr(tx_w_buffer) <= std_logic_vector(to_unsigned(tx_write_ptr,DATA_WIDTH));
                tx_buf_data_in(tx_w_buffer) <= data_from_user;
            end if;
        end if;
    end process;

rx_buffer_control_reads : process(clk, rst_n)
    variable user_rx_read_valid : std_logic;
    variable rx_rd_ptr_var : integer range 0 to BUFFER_DEPTH := 0;
      begin
        if (rst_n = '0') then
            rx_read_ptr <= 0;
            rx_rd_ptr_var := 0;
            user_rx_read_valid := '0';
            rx_buffer_empty <= "11";--both buffers start empty after reset
            rx_buffer_data_valid <= '0';
            rx_buffer_read_select <= 0;
        elsif (rising_edge(clk)) then
            if (rx_write_ptr > 0) then
                if (rx0_locked = '1') then
                    rx_buffer_empty(0) <= '0';
                elsif (rx1_locked = '1') then
                    rx_buffer_empty(1) <= '0';
                end if;
            end if;

            if (rx0_locked = '0' and rx_buffer_empty(0) = '0') then
                rx_buffer_read_select <= 0;
                user_rx_read_valid := '1';
            elsif (rx1_locked = '0' and rx_buffer_empty(1) = '0') then
                rx_buffer_read_select <= 1;
                user_rx_read_valid := '1';
            else
                user_rx_read_valid := '0';
            end if;

            if (user_command(2) = '1' and user_rx_read_valid = '1') then
                if (rx_read_ptr < BUFFER_DEPTH) then
                    rx_read_ptr <= rx_read_ptr + 1;
                    rx_rd_ptr_var := rx_rd_ptr_var + 1;
                    rx_buffer_data_valid <= '1';
                else
                    rx_read_ptr <= 0;
                    rx_rd_ptr_var := 0;
                    rx_buffer_empty(rx_buffer_read_select) <= '1';
                    rx_buffer_data_valid <= '0';
                end if;
            end if;
        end if;
    end process;

--=================================================================================================================
--logic and processes to handle rx read RAM access
    rx_buffer_reader : process (rst_n, pause, rx_read_ptr)
      begin
        if (rst_n = '0') then
            rx_read_addr <= (others => '0');
        elsif (rx_read_ptr < BUFFER_DEPTH) then
            rx_read_addr <= std_logic_vector(to_unsigned(rx_read_ptr,DATA_WIDTH));
        else
            rx_read_addr <= (others => '0');
        end if;
    end process;

    switch_data_to_user : process(rst_n, rx_buf_data_out, rx_buffer_data_valid, rx_buffer_read_select)
        begin
        if (rst_n = '0') then
            data_to_user <= (others => '0');
        elsif (rx_buffer_data_valid = '1') then
            data_to_user <= rx_buf_data_out(rx_buffer_read_select);
        else
            data_to_user <= (others => '0');
        end if;
    end process;
--=================================================================================================================

--============================================================================
    link_fis_type <= data_from_link(7 downto 0);
--============================================================================
    user_addr_proc : process(read_error, write_error, error_address, last_read_address)
      begin
        if (read_error = '1' or write_error = '1') then
            address_to_user <= error_address;
        else
            address_to_user <= last_read_address(rx_buffer_read_select);
        end if;
    end process;

    --update status vectors
    --device_ready <= not status_from_link(c_l_comm_err); --this bit is PhyRdyn, negated version is phy ready. Indicates if SSD has been initialized correctly
    status_to_user(DEVICE_READY_INDEX) <= '1' when (transport_ready = '1' and status_from_link(c_l_comm_err) = '0') else '0';

    update_status : process(current_state, tx_buffer_full, rx_buffer_full, rx_buffer_empty, tx_buffer_empty, rx0_locked, rx1_locked,tx0_locked,tx1_locked)
      begin
        if (((tx_buffer_full(0) = '0' and tx0_locked = '0') or (tx_buffer_full(1) = '0' and tx1_locked = '0')) and current_state = transport_idle) then
            status_to_user(WRITE_VALID_INDEX) <= '1';
        else
            status_to_user(WRITE_VALID_INDEX) <= '0';
        end if;
        if (((rx_buffer_empty(0) = '1' and rx0_locked = '0') or (rx_buffer_empty(1) = '1' and rx1_locked = '0')) and current_state = transport_idle) then
            status_to_user(SEND_READ_VALID_INDEX) <= '1';
        else
            status_to_user(SEND_READ_VALID_INDEX) <= '0';
        end if;
        if ((rx_buffer_empty(0) = '0' and rx0_locked = '0') or (rx_buffer_empty(1) = '0' and rx1_locked = '0')) then
            status_to_user(RETRIEVE_READ_VALID_INDEX) <= '1';
        else
            status_to_user(RETRIEVE_READ_VALID_INDEX) <= '0';
        end if;
    end process;

    status_to_user(WRITE_ERROR_INDEX) <= write_error;
    status_to_user(READ_ERROR_INDEX) <= read_error;

    --status assignments
    link_error <= status_from_link(c_l_transmit_bad);
    link_rdy <= status_from_link (5);
    pause <= status_from_link(6);
    data_from_link_valid <= status_from_link(7);
    status_to_link <= "0" & rx_from_link_ready & tx_to_link_request & "00001";
end architecture;