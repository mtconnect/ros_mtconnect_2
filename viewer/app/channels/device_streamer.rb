require 'long_pull'
require 'thwait'
require 'rexml/document'

class DeviceStreamer
  def initialize(device)
    @device = device
  end
  
  def logger
    Rails.logger
  end

  def http_client(url)
    dest = URI.parse(url)
    path = dest.path
    path += '/' unless path[-1] == ?/
    http = Net::HTTP.new(dest.host, dest.port)
    http.open_timeout = 30
    http.read_timeout = 20
    [http, path]
  end

  def stream_changes(xml)
    @device.parse_streams(xml) do |event|
      puts "Sending event: #{@device.name}_#{event.id} #{event.value}"
      ActionCable.server.broadcast('mtc_events_channel',
                                   id: "#{@device.name}_#{event.id}",
                                   name: @device.name,
                                   component: event.component,
                                   raw_id: event.id,
                                   item: event.item,
                                   value: event.value)
    end      
  end

  def initialize_stream(client, path)
    response = client.get("#{path}current")
    if Net::HTTPOK === response
      xml = response.body
      doc = REXML::Document.new(xml)
      header = doc.elements['//Header']
      nxt = header.attributes['nextSequence']      
      stream_changes(xml)

      nxt
    else
      logger.error "Agent returned status: #{response}"
      nil
    end
  end
  
  def subscribe
    client, path = http_client(@device.url)
    nxt = initialize_stream(client, path)
    
    path << "sample?from=#{nxt}&interval=100&count=1000"
    logger.info "Requesting: #{path} for #{@device.name} at #{nxt}"

    puller = LongPull.new(client)
    puller.long_pull(path) do |xml|
      stream_changes(xml)
    end
  end

  def self.start
    devices = Device.devices
    threads = devices.map do |device|
      Thread.new {
        streamer = DeviceStreamer.new(device)
        streamer.subscribe
      }
    end
    ThreadsWait.all_waits(*threads)
  end
  
end
