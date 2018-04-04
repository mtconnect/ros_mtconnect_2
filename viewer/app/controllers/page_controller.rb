class PageController < ApplicationController
  skip_before_action :verify_authenticity_token

  def index
    @devices = Device.devices
  end

  def update_mode
    p params
  end
end
