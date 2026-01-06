defmodule Nerves.IO.RC522.Application do
  @moduledoc false

  use Application

  def start(_type, _args) do
    children = [
      # Start the RC522 worker (replace with your actual worker module if different)
      {Nerves.IO.RC522.Worker, []}
    ]

    opts = [strategy: :one_for_one, name: Nerves.IO.RC522.Supervisor]
    Supervisor.start_link(children, opts)
  end
end
