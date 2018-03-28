
class MtConnectUpdatesChannel < ApplicationCable::Channel
  def self.logger
    Rails.logger
  end
  
  def subscribed
    stream_from "mt_connect_updates_channel"
  end

  def unsubscribed
    # Any cleanup needed when channel is unsubscribed
  end
end
