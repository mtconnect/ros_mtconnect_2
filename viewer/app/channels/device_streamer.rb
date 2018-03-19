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

  def initialize_stream(client, path)
    response = client.get("#{path}current")
    if Net::HTTPOK === response
      doc = REXML::Document.new(response.body)
      header = doc.elements['//Header']
      p header
      header.attributes['nextSequence']
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
      @device.parse_streams(xml) do |event|
        puts "Sending event: #{@device.name}_#{event.id} #{event.value}"
        ActionCable.server.broadcast('mt_connect_updates_channel',
                                     id: "#{@device.name}_#{event.id}",
                                     value: event.value)
      end
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
