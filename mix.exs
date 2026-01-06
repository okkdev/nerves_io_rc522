defmodule NervesIoRc522.Mixfile do
  use Mix.Project

  def project do
    [
      app: :nerves_io_rc522,
      version: "0.2.0",
      elixir: "~> 1.17",
      name: "nerves_io_rc522",
      description: description(),
      package: package(),
      source_url: "https://github.com/arjan/nerves_io_rc522",
      compilers: Mix.compilers(),
      build_embedded: Mix.env() == :prod,
      start_permanent: Mix.env() == :prod,
      deps: deps()
    ]
  end

  def application do
    [
      applications: [:logger]
    ]
  end

  defp description do
    """
    Elixir access to the RC522 RFID reader module over SPI
    """
  end

  defp package do
    %{
      files: ["lib", "mix.exs", "README.md", "LICENSE"],
      maintainers: ["Arjan Scherpenisse"],
      licenses: ["Apache-2.0"],
      links: %{"GitHub" => "https://github.com/arjan/nerves_io_rc522"}
    }
  end

  defp deps do
    []
  end
end
