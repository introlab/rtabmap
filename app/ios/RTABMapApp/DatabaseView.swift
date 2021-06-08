/**
 * Copyright (c) 2017 Razeware LLC
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * Notwithstanding the foregoing, you may not use, copy, modify, merge, publish,
 * distribute, sublicense, create a derivative work, and/or sell copies of the
 * Software in any work that is designed, intended, or marketed for pedagogical or
 * instructional purposes related to programming, coding, application development,
 * or information technology.  Permission for such use, copying, modification,
 * merger, publication, distribution, sublicensing, creation of derivative works,
 * or sale is expressly withheld.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */


import UIKit

protocol DatabaseViewDelegate: class {
    func databaseShared(databaseURL: URL)
    func databaseRenamed(databaseURL: URL)
    func databaseDeleted(databaseURL: URL)
}

class DatabaseView: UIView {
  
  private var coverImageView: UIImageView!
  private var indicatorView: UIActivityIndicatorView!
  private var valueObservation: NSKeyValueObservation!
  private var textLabel: UILabel!
  private var databaseURL: URL!
  public var delegate: DatabaseViewDelegate!
  
  required init?(coder aDecoder: NSCoder) {
    super.init(coder: aDecoder)
    commonInit(databasePath: "NA")
  }
  
    init(frame: CGRect, databaseURL: URL) {
        super.init(frame: frame)
        self.databaseURL = databaseURL
        commonInit(databasePath: databaseURL.path)
        DispatchQueue.global().async {
            let downloadedImage = getPreviewImage(databasePath: databaseURL.path)
          DispatchQueue.main.async {
            self.coverImageView.image = downloadedImage
          }
        }
    }

  private func commonInit(databasePath: String) {
    
    // Setup the background
    backgroundColor = .black
    // Create the cover image view
    coverImageView = UIImageView()
    coverImageView.translatesAutoresizingMaskIntoConstraints = false
    
    valueObservation = coverImageView.observe(\.image, options: [.new]) { [unowned self] observed, change in
      if change.newValue is UIImage {
        self.indicatorView.stopAnimating()
      }
    }
    
    //Text Label
    textLabel = UILabel()
    textLabel.numberOfLines = 0 
    textLabel.text  =
        URL(fileURLWithPath: databasePath).lastPathComponent + "\n" +
        URL(fileURLWithPath: databasePath).fileSizeString + "\n" +
        (try! URL(fileURLWithPath: databasePath).resourceValues(forKeys: [.contentModificationDateKey]).contentModificationDate!.getFormattedDate(format: "yyyy-MM-dd HH:mm:ss"))
    textLabel.textColor = .white
    textLabel.textAlignment = .left

    //Stack View
    let stackView   = UIStackView()
    stackView.axis  = NSLayoutConstraint.Axis.horizontal
    stackView.distribution  = UIStackView.Distribution.fill
    stackView.alignment = UIStackView.Alignment.center
    stackView.spacing   = 8.0

    stackView.addArrangedSubview(coverImageView)
    stackView.addArrangedSubview(textLabel)
    stackView.translatesAutoresizingMaskIntoConstraints = false

    addSubview(stackView)

    // Create the indicator view
    indicatorView = UIActivityIndicatorView()
    indicatorView.translatesAutoresizingMaskIntoConstraints = false
    indicatorView.style = .whiteLarge
    indicatorView.startAnimating()
    addSubview(indicatorView)
    
    NSLayoutConstraint.activate([
      stackView.leftAnchor.constraint(equalTo: self.leftAnchor),
      stackView.rightAnchor.constraint(equalTo: self.rightAnchor),
      stackView.topAnchor.constraint(equalTo: self.topAnchor),
      stackView.bottomAnchor.constraint(equalTo: self.bottomAnchor),
      stackView.leadingAnchor.constraint(equalTo: self.leadingAnchor),
      coverImageView.widthAnchor.constraint(equalToConstant: 180 - 5),
      indicatorView.centerXAnchor.constraint(equalTo: coverImageView.centerXAnchor),
      indicatorView.centerYAnchor.constraint(equalTo: coverImageView.centerYAnchor)
      ])
    
    let interaction = UIContextMenuInteraction(delegate: self)
    self.addInteraction(interaction)
  }
    
    func highlightDatabase(_ didHighlightView: Bool) {
      if didHighlightView == true {
        backgroundColor = .white
        textLabel.textColor = .black
      } else {
        backgroundColor = .black
        textLabel.textColor = .white
      }
    }
}

extension URL {
    var attributes: [FileAttributeKey : Any]? {
        do {
            return try FileManager.default.attributesOfItem(atPath: path)
        } catch let error as NSError {
            print("FileAttribute error: \(error)")
        }
        return nil
    }

    var fileSize: UInt64 {
        return attributes?[.size] as? UInt64 ?? UInt64(0)
    }

    var fileSizeString: String {
        return ByteCountFormatter.string(fromByteCount: Int64(fileSize), countStyle: .file)
    }

    var creationDate: Date? {
        return attributes?[.creationDate] as? Date
    }
}

extension DatabaseView: UIContextMenuInteractionDelegate {
               
      func contextMenuInteraction(_ interaction: UIContextMenuInteraction, configurationForMenuAtLocation location: CGPoint) -> UIContextMenuConfiguration? {
        return UIContextMenuConfiguration(identifier: nil, previewProvider: nil) { suggestedActions in
           
            // Create an action for sharing
            let share = UIAction(title: "Share", image: UIImage(systemName: "square.and.arrow.up")) { action in
                // Show system share sheet
                self.delegate?.databaseShared(databaseURL: self.databaseURL)
            }
    
            // Create an action for renaming
            let rename = UIAction(title: "Rename", image: UIImage(systemName: "square.and.pencil")) { action in
                // Perform renaming
                self.delegate?.databaseRenamed(databaseURL: self.databaseURL)
            }
    
            // Here we specify the "destructive" attribute to show that it’s destructive in nature
            let delete = UIAction(title: "Delete", image: UIImage(systemName: "trash"), attributes: .destructive) { action in
                // Perform delete
                self.delegate?.databaseDeleted(databaseURL: self.databaseURL)
            }
    
            // Create and return a UIMenu with all of the actions as children
            return UIMenu(title: "", children: [share, rename, delete])
        }
    }
}
