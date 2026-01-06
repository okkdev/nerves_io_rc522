defmodule Nerves.IO.RC522.Worker do
  use GenServer

  def start_link(_opts) do
    GenServer.start_link(__MODULE__, %{})
  end

  def init(state) do
    {:ok, ctx} = RC522Elixir.start_link()
    RC522Elixir.pcd_reset(ctx)
    RC522Elixir.antenna_on(ctx)
    schedule_poll()
    {:ok, %{ctx: ctx}}
  end

  def handle_info(:poll, %{ctx: ctx} = state) do
    case RC522Elixir.find_tag(ctx) do
      {:ok, _card_type} ->
        case RC522Elixir.select_tag_sn(ctx) do
          {:ok, sn, sn_len} ->
            uid_str =
              Enum.map_join(sn, "", fn b ->
                :io_lib.format("~2.16.0B", [b]) |> List.to_string()
              end)

            Logger.info("Tag UID: #{uid_str}")
            # You can send this to another process or handle as needed
            RC522Elixir.pcd_halt(ctx)

          _ ->
            :noop
        end

      _ ->
        :noop
    end

    schedule_poll()
    {:noreply, state}
  end

  defp schedule_poll, do: Process.send_after(self(), :poll, 50)
end
