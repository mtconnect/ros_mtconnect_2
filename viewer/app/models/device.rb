require 'rexml/document'

class Device
  attr_accessor :display_name, :url, :name

  def self.devices
    return @@devices
  end

  def initialize(name, url)
    @display_name, @url = name, url
  end

  def logger
    Rails.logger
  end

  devices = YAML::load(File.read(File.join(Rails.application.root, "config", "devices.yml")))
  @@devices = devices['devices'].map { |d| Device.new(d['name'], d['url']) }

  class DataValue
    attr_reader :component, :component_name, :item, :id, :name, :sub_type, :value 
    def initialize(component, component_name, item, id, name, sub_type, value)
      @component, @component_name, @item, @id, @name, @sub_type, @value=
        component, component_name, item, id, name, sub_type, value
      @value.strip!
      if @value && @value.length > 20
        @value = "#{@value[0..20]}..."
      end
      if @value =~ /^[0-9\.]+$/
        @value = @value.to_f.to_s
      end
    end
  end

  class Condition < DataValue
    attr_reader :value_type
    def initialize(component, component_name, item, id, name, sub_type, value,
        value_type)
      super(component, component_name, item, id, name, sub_type, value)
      @value_type = value_type
    end
  end
  
  class Component 
    attr_accessor :name, :nativeName, :type, :values
    def initialize(name, nativeName, type)
      @name, @nativeName, @type = name, nativeName, type
      @values = []
    end
  end
  
  def get_data
    @asset = nil
    dest = URI.parse(self.url)
    client = response = nil
    Timeout::timeout(3) do
      Net::HTTP.start(dest.host, dest.port) do |client|
        response = client.get("#{dest.path}/current")
      end
    end
    if Net::HTTPOK === response
      components = parse_streams(response.body)
      
      result = Hash.new { |h, v| h[v] = [] }
      components.delete_if { |c| c.values.empty? }
      components.each { |c| result[c.type] << c }
      result.each { |k, v| v.sort_by { |c| c.name} }
      result
    else
      logger.error "Response from server #{response}"
      []
    end

  rescue Timeout::Error
    logger.error "Request to #{self.url} timed out"
    []

  rescue Errno::ECONNREFUSED, SocketError
    logger.error "Could not connect to #{self.url}"
    []
  
  rescue
    logger.error "#{$!.class}: Unexpected error: #{$!}"
    logger.error $!.backtrace.join("\n")
    []
  end

  def parse_streams(xml)
    document = REXML::Document.new(xml)
    components = []

    device = document.elements['//DeviceStream']
    unless device
      logger.debug "No device stream"
      return []
    end
    @name = device.attributes['name']
    
    document.each_element('//ComponentStream') do |component|
      comp_attrs = component.attributes
      comp = Component.new(comp_attrs['name'] || comp_attrs['componentId'], 
                           comp_attrs['nativeName'], 
                           comp_attrs['component'])
      
      components << comp
      component.each_element('Events/*|Condition/*') do |value|
        value_attrs = value.attributes
        if value.parent.name == 'Condition'
          cond = Condition.new(comp_attrs['component'], comp_attrs['name'],
                               value.name, value_attrs['dataItemId'], value_attrs['name'],
                               value_attrs['subType'], value.text || "", value_attrs['type'])
          yield cond if block_given?
          comp.values << cond
        else
          event = DataValue.new(comp_attrs['component'], comp_attrs['name'], value.name,
                                value_attrs['dataItemId'], value_attrs['name'],
                                value_attrs['subType'], value.text || "")
          yield event if block_given?
          comp.values << event
        end
      end
    end

    components
  end
  
  def get_asset(asset_id)
    dest = URI.parse(self.url)
    response = asset = nil
    begin
      Timeout::timeout(5) do 
        Net::HTTP.start(dest.host, dest.port) do |client|
          response = client.get("/asset/#{asset_id}")
        end
      end  
      if Net::HTTPOK === response
        asset = REXML::Document.new(response.body)
        asset = nil if asset.root.name != "MTConnectAssets"
      end
    rescue Timeout::Error
    rescue Exception
      logger.error "Error getting asset: #{$!}"
    end
    asset
  end
  

  def has_task?
    self.task?
  end
  
end
