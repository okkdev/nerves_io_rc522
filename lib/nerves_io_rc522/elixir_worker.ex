defmodule Nerves.IO.RC522 do
  use GenServer
  require Logger

  def start_link(opts) do
    GenServer.start_link(__MODULE__, opts, name: __MODULE__)
  end

  def get_context do
    GenServer.call(__MODULE__, :get_context)
  end

  # to run diagnostics on the RC522 with Nerves.IO.RC522.run_diagnostics()
  def run_diagnostics do
    GenServer.call(__MODULE__, :run_diagnostics)
  end

  def init(opts) do
    Logger.info("RC522 worker starting - initializing SPI and GPIO")

    # Extract callback if provided
    callback =
      case opts do
        {module, function} -> {module, function}
        _ -> nil
      end

    {:ok, ctx} = RC522Elixir.start_link()
    RC522Elixir.pcd_reset(ctx)
    RC522Elixir.antenna_on(ctx)
    Logger.info("RC522 worker initialized successfully")
    schedule_poll()
    {:ok, %{ctx: ctx, callback: callback}}
  end

  def handle_call(:get_context, _from, %{ctx: ctx} = state) do
    {:reply, ctx, state}
  end

  def handle_call(:run_diagnostics, _from, %{ctx: ctx} = state) do
    result = RC522Elixir.run_diagnostics(ctx)
    {:reply, result, state}
  end

  def handle_info(:poll, %{ctx: ctx, callback: callback} = state) do
    case RC522Elixir.find_tag(ctx) do
      {:ok, card_type} ->
        Logger.debug("Tag detected, card type: #{inspect(card_type)}")

        case RC522Elixir.select_tag_sn(ctx) do
          {:ok, sn, _sn_len} ->
            uid_str =
              Enum.map_join(sn, "", fn b ->
                :io_lib.format("~2.16.0B", [b]) |> List.to_string()
              end)

            Logger.info("Tag UID: #{uid_str}")

            # Call the callback if provided
            if callback do
              {module, function} = callback
              apply(module, function, [uid_str])
            end

          # Note: We do not halt the tag here to allow for continuous reading. If you want to halt after reading, you can uncomment the line below.
          # RC522Elixir.pcd_halt(ctx)

          {:error, reason} ->
            Logger.warning("Failed to select tag serial number: #{inspect(reason)}")
        end

      {:error, :notag} ->
        # Ignore no tag present
        :noop

      {:error, reason} ->
        Logger.debug("Tag detection error: #{inspect(reason)}")

      _ ->
        :noop
    end

    schedule_poll()
    {:noreply, state}
  end

  defp schedule_poll, do: Process.send_after(self(), :poll, 50)
end
