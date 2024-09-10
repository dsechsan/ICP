from PIL import Image, ImageDraw, ImageFont
import imageio
import os
import re

# Configuration settings
input_folder = "/Users/dsechs/Library/CloudStorage/OneDrive-UCSanDiego/Desktop/Cpp/pointclouds/frames/"
output_gif_path = "output60.gif"

# Font paths and settings
font_path = "/System/Library/Fonts/Supplemental/Microsoft Sans Serif.ttf"
font_size_iter = 55
font_size_title = 70
font_color_iter = (0, 0, 0)
font_color_title = (0, 100, 0)

font_size_legend = 40
font_color_source = (0, 0, 128)
font_color_current = (230, 230, 0)
font_color_target = (128, 0, 0)


def add_text_to_image(image_path, iter_text, title, source, target, current):
    """
    Adds iteration number, title, and legend text to an image.

    :param image_path: Path to the input image.
    :param iter_text: Text for the iteration number.
    :param title: Title text to add to the image.
    :param source: Legend text for the source.
    :param target: Legend text for the target.
    :param current: Legend text for the current.
    :return: Annotated PIL image object.
    """
    image = Image.open(image_path)
    draw = ImageDraw.Draw(image)

    try:
        font_iter = ImageFont.truetype(font_path, font_size_iter)
        font_title = ImageFont.truetype(font_path, font_size_title)
        font_legend = ImageFont.truetype(font_path, font_size_legend)
    except IOError:
        font_iter = ImageFont.load_default()
        font_title = ImageFont.load_default()
        font_legend = ImageFont.load_default()

    # Positions for text
    iter_text_position = (200, 300)
    title_text_position = (550, 30)
    legend_source_pos = (230, 400)
    legend_target_pos = (230, 475)
    legend_current_pos = (230, 550)

    # Draw text on the image
    draw.text(iter_text_position, iter_text, font=font_iter, fill=font_color_iter)
    draw.text(title_text_position, title, font=font_title, fill=font_color_title)
    draw.text(legend_source_pos, source, font=font_legend, fill=font_color_source)
    draw.text(legend_target_pos, target, font=font_legend, fill=font_color_target)
    draw.text(legend_current_pos, current, font=font_legend, fill=font_color_current)

    return image


def extract_number(filename):
    """
    Extracts numbers from a filename for sorting.

    :param filename: Filename from which to extract numbers.
    :return: Extracted number or infinity if no number is found.
    """
    match = re.search(r'\d+', filename)
    return int(match.group()) if match else float('inf')


def create_gif(input_folder, output_gif_path):
    """
    Creates a GIF from images in a folder, annotating each with text.

    :param input_folder: Folder containing input images.
    :param output_gif_path: Path to save the output GIF.
    """
    images = []
    for filename in sorted(os.listdir(input_folder), key=extract_number):
        if filename.endswith(".png"):
            frame_number = extract_number(filename)
            image_path = os.path.join(input_folder, filename)

            # Add annotations to the image
            annotated_image = add_text_to_image(
                image_path,
                f"Iteration: {frame_number}",
                "ICP Visualization",
                "Source",
                "Target",
                "Current"
            )
            images.append(annotated_image)

    # Save the annotated images as a GIF
    if images:
        images[0].save(
            output_gif_path,
            save_all=True,
            append_images=images[1:],  # Exclude the first image since it's saved directly
            duration=200,  # Duration between frames in milliseconds
            loop=0  # 0 means loop indefinitely
        )
        print(f"GIF created successfully: {output_gif_path}")
    else:
        print("No images found to create GIF.")


def main():
    """
    Main function to run the GIF creation process.
    """
    create_gif(input_folder, output_gif_path)


if __name__ == "__main__":
    main()