class PageController < ApplicationController
  def index
    @devices = Device.devices.map { |d| [d.display_name, d.name, d.get_data] }
  end
end
