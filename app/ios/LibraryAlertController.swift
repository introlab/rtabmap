//
//  LibbraryAlertViewController.swift
//  RTABMapApp
//
//  Created by Mathieu Labbe on 2026-10-24.
//

import UIKit

class LibraryAlertController: UIAlertController {
    
    var containerVC: UIViewController?
    
    override func viewDidLoad() {
        super.viewDidLoad()
        updateAlertSize(with: view.bounds.size)
    }
    
    // This triggers automatically whenever the device rotates
    override func viewWillTransition(to size: CGSize, with coordinator: UIViewControllerTransitionCoordinator) {
        super.viewWillTransition(to: size, with: coordinator)
        
        coordinator.animate(alongsideTransition: { _ in
            // Recalculate size based on the new orientation dimensions
            self.updateAlertSize(with: size)
        }, completion: nil)
    }
    
    private func updateAlertSize(with screenSize: CGSize) {
        guard let containerVC = containerVC else { return }

        // Set width to 0 or 270 here. iOS overrides the width parameter entirely,
        // but it is still safe to pass the maximum expected bound.
        containerVC.preferredContentSize = CGSize(width: 270, height: screenSize.height * 0.65)
    }
}
