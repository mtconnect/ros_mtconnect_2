
class MtcEventsChannel < ApplicationCable::Channel
  def subscribed
    stream_from "mtc_events_channel"
  end

  def unsubscribed
    # Any cleanup needed when channel is unsubscribed
  end
end
