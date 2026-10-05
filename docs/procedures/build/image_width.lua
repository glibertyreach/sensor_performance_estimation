-- Pandoc filter: give every image an explicit width so Word lays it out at
-- the text width of a US Letter page with 1 inch margins. The height follows
-- from the image's aspect ratio.
local TEXT_WIDTH = "6.5in"

function Image(img)
  img.attributes.width = TEXT_WIDTH
  return img
end
