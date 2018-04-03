module PageHelper
  def format_element(element, prefix)
    text = []
    if element.text.nil? or element.text.strip.empty?
      text << "#{prefix}#{element.name}:"
      element.each_element do |child|
        text << format_element(child, prefix + '  ')
      end      
    else
      text = "#{prefix}#{element.name}: #{element.text.strip}"
    end
    p text
    text
  end
  
  def format_asset(device, asset)
    haml_tag(:pre, device.format_asset(asset),
             id: "#{device.name}_#{asset.attributes['assetId']}")
  end
end
