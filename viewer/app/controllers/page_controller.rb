class PageController < ApplicationController
  skip_before_action :verify_authenticity_token

  def index
    @devices = Device.devices.map { |d| [d.display_name, d.name, d.get_data] }
  end

  def update_mode
    p params
  end
end
