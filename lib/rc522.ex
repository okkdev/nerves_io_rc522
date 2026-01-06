defmodule RC522Elixir do
  @moduledoc """
  Minimal RC522 driver using Circuits.SPI and Circuits.GPIO.
  """

  import Bitwise
  require Logger

  alias Circuits.SPI
  alias Circuits.GPIO

  @spi_bus "spidev0.0"
  # GPIO25 (physical pin 22)
  @rst_gpio 25

  # Constants
  @bit_framing_reg 0x0D
  @coll_reg 0x0E
  @status2_reg 0x08
  @com_ien_reg 0x02
  @com_irq_reg 0x04
  @fifo_level_reg 0x0A
  @command_reg 0x01
  @fifo_data_reg 0x09
  @error_reg 0x06
  @control_reg 0x0C
  @div_irq_reg 0x05
  @crc_result_reg_l 0x22
  @crc_result_reg_m 0x21
  @tx_control_reg 0x14
  @t_mode_reg 0x2A
  @t_prescaler_reg 0x2B
  @t_reload_reg_l 0x2C
  @t_reload_reg_h 0x2D
  @tx_ask_reg 0x15
  @mode_reg 0x11
  @rx_threshold_reg 0x18
  @rf_cfg_reg 0x26
  @gs_n_reg 0x27
  @cwgs_cfg_reg 0x28

  @pcd_idle 0x00
  @pcd_authent 0x0E
  @pcd_transceive 0x0C
  @pcd_calccrc 0x03
  @pcd_resetphase 0x0F

  @picc_read 0x30
  @picc_write 0xA0
  @picc_halt 0x50
  @picc_reqidl 0x26
  @picc_anticoll1 0x93
  @picc_anticoll2 0x95
  @picc_anticoll3 0x97

  @tag_ok 0
  @tag_err 1
  @tag_notag 2
  @tag_collision 3
  # @tag_errcrc 4

  @max_rlen 18
  # end constants

  # Open SPI and RST GPIO
  def start_link() do
    {:ok, spi} = SPI.open(@spi_bus)
    {:ok, rst} = GPIO.open(@rst_gpio, :output)
    # Set RST high
    GPIO.write(rst, 1)
    {:ok, %{spi: spi, rst: rst}}
  end

  # Write to RC522 register
  def write_reg(%{spi: spi}, address, value) do
    # Address: 7 bits, value: 8 bits
    # Write: MSB=0, so ((address <<< 1) &&& 0x7E)
    data = <<address <<< 1 &&& 0x7E, value>>
    SPI.transfer(spi, data)
    :ok
  end

  # Read from RC522 register
  def read_reg(%{spi: spi}, address) do
    # Read: MSB=1, so ((address <<< 1) &&& 0x7E) | 0x80
    data = <<(address <<< 1 &&& 0x7E) ||| 0x80, 0x00>>
    {:ok, <<_addr, value>>} = SPI.transfer(spi, data)
    value
  end

  # Example: RC522 reset sequence
  def reset(ctx) do
    # CommandReg, PCD_RESETPHASE
    write_reg(ctx, 0x01, 0x0F)
    Process.sleep(10)
    # ... add more register writes as needed
  end

  # Example: Antenna on
  def antenna_on(ctx) do
    # TxControlReg
    val = read_reg(ctx, 0x14)

    if (val &&& 0x03) == 0 do
      write_reg(ctx, 0x14, val ||| 0x03)
    end
  end

  # Example: Antenna off
  def antenna_off(ctx) do
    # TxControlReg
    val = read_reg(ctx, 0x14)
    write_reg(ctx, 0x14, val &&& Bitwise.bnot(0x03))
  end

  def set_bit_mask(ctx, reg, mask) do
    tmp = read_reg(ctx, reg)
    write_reg(ctx, reg, tmp ||| mask)
  end

  def clear_bit_mask(ctx, reg, mask) do
    tmp = read_reg(ctx, reg)
    write_reg(ctx, reg, tmp &&& Bitwise.bnot(mask))
  end

  def pcd_request(ctx, req_code) do
    # Set BitFramingReg to 0x07
    write_reg(ctx, @bit_framing_reg, 0x07)

    # Prepare buffer
    uc_com_mf522_buf = [req_code]

    # Transceive: send req_code, receive response
    {status, uc_com_mf522_buf, un_len} =
      pcd_com_mf522(ctx, @pcd_transceive, uc_com_mf522_buf)

    cond do
      status == @tag_ok and un_len == 0x10 ->
        tag_type = Enum.take(uc_com_mf522_buf, 2)
        {:ok, tag_type}

      status == @tag_collision ->
        {:collision, uc_com_mf522_buf}

      status != @tag_notag ->
        {:error, :tag_err}

      true ->
        {:error, :unknown}
    end
  end

  def pcd_anticoll(ctx, cascade) do
    # Set BitFramingReg to 0x00
    write_reg(ctx, @bit_framing_reg, 0x00)

    pass = 32
    collbits = 0
    i = 0
    status = nil
    uc_com_mf522_buf = []

    # Loop for anticollision
    result =
      Enum.reduce_while(1..pass, {i, collbits, nil, []}, fn _,
                                                            {i, collbits, status,
                                                             uc_com_mf522_buf} ->
        buf = [cascade, 0x20 + collbits] ++ List.duplicate(0, i)
        {new_status, new_buf, un_len} = pcd_com_mf522(ctx, @pcd_transceive, buf)

        if new_status == @tag_collision do
          collbits = read_reg(ctx, @coll_reg) &&& 0x1F
          collbits = if collbits == 0, do: 32, else: collbits
          i = div(collbits - 1, 8) + 1

          # Set the collision bit
          buf = List.update_at(buf, i - 1, fn val -> val ||| 1 <<< rem(collbits - 1, 8) end)
          # Update buffer shifting (mimic C logic, may need adjustment)
          buf = List.replace_at(buf, 5, buf[3])
          buf = List.replace_at(buf, 4, buf[2])
          buf = List.replace_at(buf, 3, buf[1])
          buf = List.replace_at(buf, 2, buf[0])

          # Set BitFramingReg to (collbits % 8)
          write_reg(ctx, @bit_framing_reg, rem(collbits, 8))
          {:cont, {i, collbits, new_status, buf}}
        else
          {:halt, {i, collbits, new_status, new_buf}}
        end
      end)

    {i, collbits, status, uc_com_mf522_buf} = result

    # Check result
    if status == @tag_ok do
      # Get serial number and check
      snr = Enum.take(uc_com_mf522_buf, 4)
      snr_check = Enum.reduce(snr, 0, &Bitwise.bxor/2)
      snr_check_val = Enum.at(uc_com_mf522_buf, 4)

      if snr_check != snr_check_val do
        {:error, :tag_err}
      else
        {:ok, snr}
      end
    else
      {:error, status}
    end
  end

  def pcd_select(ctx, cascade, p_snr) do
    # Prepare buffer
    buf = List.duplicate(0, 9)
    buf = List.replace_at(buf, 0, cascade)
    buf = List.replace_at(buf, 1, 0x70)
    buf = List.replace_at(buf, 6, Enum.reduce(p_snr, 0, &Bitwise.bxor/2))
    buf = List.replace_at(buf, 2, Enum.at(p_snr, 0))
    buf = List.replace_at(buf, 3, Enum.at(p_snr, 1))
    buf = List.replace_at(buf, 4, Enum.at(p_snr, 2))
    buf = List.replace_at(buf, 5, Enum.at(p_snr, 3))

    # Calculate CRC
    buf = calulate_crc(ctx, buf, 7)

    # Clear Status2Reg bit 0x08
    clear_bit_mask(ctx, @status2_reg, 0x08)

    {status, out_buf, un_len} = pcd_com_mf522(ctx, @pcd_transceive, buf)

    if status == @tag_ok and un_len == 0x18 do
      {:ok, out_buf}
    else
      {:error, status}
    end
  end

  def pcd_auth_state(ctx, auth_mode, addr, p_key, p_snr) do
    buf =
      [auth_mode, addr] ++
        Enum.take(p_key, 6) ++
        Enum.take(p_snr, 4)

    {status, _out_buf, _un_len} = pcd_com_mf522(ctx, @pcd_authent, buf)

    if status == @tag_ok and (read_reg(ctx, @status2_reg) &&& 0x08) != 0 do
      :ok
    else
      {:error, status}
    end
  end

  def pcd_read(ctx, addr) do
    buf = [@picc_read, addr]
    buf = calulate_crc(ctx, buf, 2)

    {status, out_buf, un_len} = pcd_com_mf522(ctx, @pcd_transceive, buf)
    crc_buff = calulate_crc(ctx, out_buf, 16)

    if status == @tag_ok and un_len == 0x90 do
      if Enum.at(crc_buff, 0) != Enum.at(out_buf, 16) or
           Enum.at(crc_buff, 1) != Enum.at(out_buf, 17) do
        {:error, :tag_err_crc}
      else
        {:ok, Enum.take(out_buf, 16)}
      end
    else
      {:error, status}
    end
  end

  def pcd_write(ctx, addr, data) do
    buf = [@picc_write, addr]
    buf = calulate_crc(ctx, buf, 2)

    {status, out_buf, un_len} = pcd_com_mf522(ctx, @pcd_transceive, buf)

    if status != @tag_ok or un_len != 4 or (Enum.at(out_buf, 0) &&& 0x0F) != 0x0A do
      {:error, status}
    else
      buf2 = Enum.take(data, 16)
      buf2 = calulate_crc(ctx, buf2, 16)

      {status2, out_buf2, un_len2} = pcd_com_mf522(ctx, @pcd_transceive, buf2)

      if status2 != @tag_ok or un_len2 != 4 or (Enum.at(out_buf2, 0) &&& 0x0F) != 0x0A do
        {:error, status2}
      else
        :ok
      end
    end
  end

  def pcd_halt(ctx) do
    buf = [@picc_halt, 0]
    buf = calulate_crc(ctx, buf, 2)
    {status, _out_buf, _un_len} = pcd_com_mf522(ctx, @pcd_transceive, buf)
    status
  end

  def calulate_crc(ctx, buf, len) do
    # Clear DivIrqReg bit 0x04
    clear_bit_mask(ctx, @div_irq_reg, 0x04)
    # Set CommandReg to PCD_IDLE
    write_reg(ctx, @command_reg, @pcd_idle)
    # Set FIFOLevelReg bit 0x80
    set_bit_mask(ctx, @fifo_level_reg, 0x80)
    # Write data to FIFO
    Enum.each(Enum.take(buf, len), fn byte -> write_reg(ctx, @fifo_data_reg, byte) end)
    # Set CommandReg to PCD_CALCCRC
    write_reg(ctx, @command_reg, @pcd_calccrc)
    # Wait for CRC calculation
    i = 0xFF
    n = 0

    {_, _} =
      Enum.reduce_while(1..i, {i, n}, fn _, {i, n} ->
        n = read_reg(ctx, @div_irq_reg)
        i = i - 1

        if i == 0 or (n &&& 0x04) != 0 do
          {:halt, {i, n}}
        else
          {:cont, {i, n}}
        end
      end)

    # Read CRC result
    crc_l = read_reg(ctx, @crc_result_reg_l)
    crc_m = read_reg(ctx, @crc_result_reg_m)
    buf ++ [crc_l, crc_m]
  end

  def pcd_reset(ctx) do
    write_reg(ctx, @command_reg, @pcd_resetphase)
    Process.sleep(10)
    clear_bit_mask(ctx, @tx_control_reg, 0x03)
    Process.sleep(10)
    set_bit_mask(ctx, @tx_control_reg, 0x03)
    write_reg(ctx, @t_mode_reg, 0x8D)
    write_reg(ctx, @t_prescaler_reg, 0x3E)
    write_reg(ctx, @t_reload_reg_l, 30)
    write_reg(ctx, @t_reload_reg_h, 0)
    write_reg(ctx, @tx_ask_reg, 0x40)
    write_reg(ctx, @mode_reg, 0x3D)
    write_reg(ctx, @rx_threshold_reg, 0x84)
    write_reg(ctx, @rf_cfg_reg, 0x68)
    write_reg(ctx, @gs_n_reg, 0xFF)
    write_reg(ctx, @cwgs_cfg_reg, 0x2F)
    :ok
  end

  def pcd_com_mf522(ctx, command, p_in) do
    com_ien_reg = @com_ien_reg
    com_irq_reg = @com_irq_reg
    fifo_level_reg = @fifo_level_reg
    command_reg = @command_reg
    fifo_data_reg = @fifo_data_reg
    error_reg = @error_reg
    control_reg = @control_reg
    bit_framing_reg = @bit_framing_reg
    pcd_authent = @pcd_authent
    pcd_transceive = @pcd_transceive

    # IRQ and waitFor values
    {irq_en, wait_for} =
      case command do
        ^pcd_authent -> {0x12, 0x10}
        ^pcd_transceive -> {0x77, 0x30}
        _ -> {0x00, 0x00}
      end

    # Set up registers
    write_reg(ctx, com_ien_reg, irq_en ||| 0x80)
    clear_bit_mask(ctx, com_irq_reg, 0x80)
    set_bit_mask(ctx, fifo_level_reg, 0x80)
    write_reg(ctx, command_reg, @pcd_idle)

    # Write data to FIFO
    Enum.each(p_in, fn byte -> write_reg(ctx, fifo_data_reg, byte) end)

    # Start command
    write_reg(ctx, command_reg, command)

    if command == @pcd_transceive do
      set_bit_mask(ctx, bit_framing_reg, 0x80)
    end

    # Wait for completion
    i = 150
    n = 0

    {i, n} =
      Enum.reduce_while(1..i, {i, n}, fn _, {i, n} ->
        # usleep(200) in C
        Process.sleep(1)
        n = read_reg(ctx, com_irq_reg)
        i = i - 1

        if i == 0 or (n &&& 0x01) != 0 or (n &&& wait_for) != 0 do
          {:halt, {i, n}}
        else
          {:cont, {i, n}}
        end
      end)

    clear_bit_mask(ctx, bit_framing_reg, 0x80)

    {status, p_out, p_out_len_bit} =
      if i != 0 do
        pcd_err = read_reg(ctx, error_reg)

        cond do
          (pcd_err &&& 0x08) != 0 ->
            {@tag_collision, [], 0}

          (pcd_err &&& 0x11) == 0 ->
            status =
              if (n &&& irq_en &&& 0x01) != 0 do
                @tag_notag
              else
                @tag_ok
              end

            if command == @pcd_transceive do
              fifo_level = read_reg(ctx, fifo_level_reg)
              last_bits = read_reg(ctx, control_reg) &&& 0x07

              p_out_len_bit =
                if last_bits != 0 do
                  (fifo_level - 1) * 8 + last_bits
                else
                  fifo_level * 8
                end

              n = if fifo_level == 0, do: 1, else: fifo_level
              n = if n > @max_rlen, do: @max_rlen, else: n

              p_out = for _idx <- 0..(n - 1), do: read_reg(ctx, fifo_data_reg)
              {status, p_out, p_out_len_bit}
            else
              {status, [], 0}
            end

          true ->
            {@tag_err, [], 0}
        end
      else
        {@tag_err, [], 0}
      end

    {status, p_out, p_out_len_bit}
  end

  def find_tag(ctx) do
    # Try to find a tag in the field
    case pcd_request(ctx, @picc_reqidl) do
      {:ok, [b0, b1]} ->
        card_type = b0 <<< 8 ||| b1
        {:ok, card_type}

      other ->
        other
    end
  end

  def select_tag_sn(ctx) do
    # Returns {:ok, sn, len} or {:error, reason}
    with {:ok, buff1} <- pcd_anticoll(ctx, @picc_anticoll1),
         {:ok, _} <- pcd_select(ctx, @picc_anticoll1, buff1) do
      cond do
        Enum.at(buff1, 0) == 0x88 ->
          sn = Enum.slice(buff1, 1, 3)

          with {:ok, buff2} <- pcd_anticoll(ctx, @picc_anticoll2),
               {:ok, _} <- pcd_select(ctx, @picc_anticoll2, buff2) do
            if Enum.at(buff2, 0) == 0x88 do
              sn2 = Enum.slice(buff2, 1, 3)

              with {:ok, buff3} <- pcd_anticoll(ctx, @picc_anticoll3),
                   {:ok, _} <- pcd_select(ctx, @picc_anticoll3, buff3) do
                sn3 = Enum.slice(buff3, 0, 4)
                {:ok, sn ++ sn2 ++ sn3, 10}
              end
            else
              sn2 = Enum.slice(buff2, 0, 4)
              {:ok, sn ++ sn2, 7}
            end
          end

        true ->
          sn = Enum.slice(buff1, 0, 4)
          {:ok, sn, 4}
      end
    else
      {:error, reason} -> {:error, reason}
    end
  end

  def read_tag_str(ctx, addr) do
    # Reads a block and returns a hex string or error
    case pcd_read(ctx, addr) do
      {:ok, data} ->
        str =
          Enum.map_join(data, "", fn b -> :io_lib.format("~2.16.0B", [b]) |> List.to_string() end)

        {:ok, str}

      {:error, :tag_err_crc} ->
        {:error, "CRC Error"}

      {:error, _} ->
        {:error, "Unknown error"}
    end
  end
end
