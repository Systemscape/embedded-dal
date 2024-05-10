#![no_std]

pub mod drivers;
pub mod test_images;

pub mod config {
    #[derive(Clone, Copy, PartialEq, Eq)]
    pub enum Orientation {
        Landscape,
        Portrait,
    }

    pub struct Dimensions {
        x: u16,
        y: u16,
    }

    impl Dimensions {
        pub const fn from(x: u16, y: u16) -> Self {
            Self { x, y }
        }
        pub const fn get_width(&self, orientation: Orientation) -> u16 {
            match orientation {
                Orientation::Landscape => self.x,
                Orientation::Portrait => self.y,
            }
        }

        pub const fn get_height(&self, orientation: Orientation) -> u16 {
            match orientation {
                Orientation::Landscape => self.y,
                Orientation::Portrait => self.x,
            }
        }
    }
}
